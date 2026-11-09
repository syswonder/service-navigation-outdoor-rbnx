import json,time
from pathlib import Path
import numpy as np
import yaml
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor,ExternalShutdownException
from rclpy.clock import Clock,ClockType
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from std_msgs.msg import Header,String
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener
from sensor_sanity_checker.images import ImageDecoder
from .geometry import observations,transform,rigid_matrix,cleanup,rasterize,ConsecutiveRoad
from .completion import complete_unknown
from .ros_utils import matrix_from_tf,cloud_message,seconds

class Projection(Node):
    def __init__(self,**kwargs):
        super().__init__('semantic_projection_node',**kwargs)
        defaults={'mask_topic':'/semantic/road_mask',
          'depth_topic':'/camera_gemini330/depth/image_raw',
          'depth_info_topic':'/camera_gemini330/depth/camera_info',
          'color_info_topic':'/camera_gemini330/color/camera_info',
          'grid_topic':'/semantic/local_grid','points_topic':'/semantic/road_points',
          'planning_frame':'','robot_base_frame':'','temporal_fixed_frame':'',
          'grid_alignment':'planning',
          'offline_completion':False,'prior_x_min':-0.3,'prior_x_max':1.3,
          'prior_half_width':0.6,'completion_kernel':9,'completion_road_ratio':0.55,
          'sync_slop':0.05,'sync_queue':60,'depth_scale':0.001,'depth_min':0.2,'depth_max':8.0,
          'ground_z_min':0.0,'ground_z_max':0.0,'ground_limits_confirmed':False,
          'resolution':0.05,'width':160,'height':160,'origin_x':-2.0,'origin_y':-4.0,
          'pixel_stride':2,'opening':False,'closing':False,'morph_kernel':3,
          'temporal_frames':1,'motion_reset_translation':0.01,'motion_reset_yaw':0.01,
          'max_data_age':1.0,'watchdog_timeout':1.5,'test_mode':False,
          'alignment_verified':False,'allow_unverified_alignment':False,
          'alignment_mode':'registered_same_pixels','mock_extrinsics_file':'','navigation_pose_topic':''}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        p=self.p
        if p['offline_completion'] and not p['test_mode']: raise ValueError('Completion prior requires explicit offline test_mode')
        if p['grid_alignment'] not in ('planning','robot'): raise ValueError('grid_alignment must be planning or robot')
        if not p['planning_frame'] or not p['robot_base_frame']: raise ValueError('Explicit planning_frame and robot_base_frame required')
        if p['resolution']<=0 or min(p['width'],p['height'])<=0: raise ValueError('Invalid grid dimensions')
        if p['mock_extrinsics_file'] and not p['test_mode']: raise ValueError('Mock extrinsics forbidden outside test_mode')
        if p['allow_unverified_alignment'] and not p['test_mode']: raise ValueError('Unverified alignment bypass requires test_mode')
        self.mock=None
        if p['mock_extrinsics_file']:
            self.mock=yaml.safe_load(Path(p['mock_extrinsics_file']).read_text())
        self.decoder=ImageDecoder();self.temporal=ConsecutiveRoad(p['temporal_frames'])
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.navigation_pose=None
        if p['navigation_pose_topic']:
            from .navigation_pose import NavigationPose
            self.navigation_pose=NavigationPose(self,p['navigation_pose_topic'])
        self.grid_pub=self.create_publisher(OccupancyGrid,p['grid_topic'],5)
        self.current_grid_pub=self.create_publisher(OccupancyGrid,'/semantic/current_grid',5)
        self.cloud_pub=self.create_publisher(PointCloud2,p['points_topic'],5)
        self.status_pub=self.create_publisher(String,'/semantic/projection_status',10)
        self.subs=[message_filters.Subscriber(self,typ,p[key],qos_profile=qos_profile_sensor_data)
                   for key,typ in [('mask_topic',Image),('depth_topic',Image),('depth_info_topic',CameraInfo),('color_info_topic',CameraInfo)]]
        self.sync=message_filters.ApproximateTimeSynchronizer(self.subs,p['sync_queue'],p['sync_slop'])
        self.sync.registerCallback(self.receive)
        self.last_success=None;self.last_stamp=None;self.anchor=None
        self.origin=np.array([p['origin_x'],p['origin_y']]);self.yaw=0.
        self.processed=0;self.previous_origin=None
        self.timer=self.create_timer(.25,self.watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Projection ready; unverified calibration/ground limits fail closed')
    def status(self,**values):
        self.status_pub.publish(String(data=json.dumps(values,allow_nan=False)))
    def matrix(self,target,source,stamp):
        if target==source: return np.eye(4)
        if self.navigation_pose and target==self.p['planning_frame'] and source==self.p['robot_base_frame']:
            return self.navigation_pose.matrix(stamp,wait_timeout=.12)
        try:
            return matrix_from_tf(self.tf.lookup_transform(target,source,Time.from_msg(stamp),timeout=Duration(seconds=.03)))
        except Exception:
            if self.mock and self.mock['target_frame']==target and self.mock['source_frame']==source:
                return rigid_matrix(self.mock['translation'],self.mock['quaternion_xyzw'])
            raise
    def publish(self,grid,points,stamp,current_grid=None):
        p=self.p;header=Header(stamp=stamp,frame_id=p['planning_frame'])
        msg=OccupancyGrid();msg.header=header;msg.info.map_load_time=stamp
        msg.info.resolution=p['resolution'];msg.info.width=p['width'];msg.info.height=p['height']
        msg.info.origin.position.x=float(self.origin[0]);msg.info.origin.position.y=float(self.origin[1])
        msg.info.origin.orientation.z=float(np.sin(self.yaw/2));msg.info.origin.orientation.w=float(np.cos(self.yaw/2))
        msg.data=grid.reshape(-1).tolist()
        self.grid_pub.publish(msg)
        if current_grid is not None:
            raw=OccupancyGrid();raw.header=header;raw.info=msg.info
            raw.data=current_grid.reshape(-1).tolist()
            self.current_grid_pub.publish(raw)
        self.cloud_pub.publish(cloud_message(points,header))
    def fail(self,reason):
        self.temporal.reset();self.anchor=None;self.last_success=None
        empty=np.full((self.p['height'],self.p['width']),-1,np.int8)
        self.publish(empty,np.empty((0,3)),self.get_clock().now().to_msg(),empty)
        self.status(ok=False,reason=reason,processed=self.processed,free_cells=0)
    def watchdog(self):
        if self.last_success is None or time.monotonic()-self.last_success>self.p['watchdog_timeout']:
            self.fail('no fresh synchronized observation / watchdog expired')
    def receive(self,mask_msg,depth_msg,di,ci):
        p=self.p
        processing_started=time.monotonic()
        try:
            if not p['alignment_verified'] and not (p['test_mode'] and p['allow_unverified_alignment']):
                raise ValueError('Alignment unverified; confirm pixel model before real operation')
            if not p['ground_limits_confirmed'] or not p['ground_z_min']<p['ground_z_max']:
                raise ValueError('Ground limits unconfirmed; inspect base-frame z histogram')
            stamps=[seconds(x.header.stamp) for x in (mask_msg,depth_msg,di,ci)]
            now=self.get_clock().now().nanoseconds*1e-9
            if min(stamps)<=0 or max(stamps)-min(stamps)>p['sync_slop'] or now-min(stamps)>p['max_data_age'] or max(stamps)-now>.1:
                raise ValueError('Unsynchronized, stale, zero or future sensor timestamps')
            if self.last_stamp is not None and stamps[1]<=self.last_stamp:
                self.last_stamp=None
                raise ValueError('Repeated/backward depth time: reset history')
            if self.last_stamp is not None and stamps[1]-self.last_stamp>p['watchdog_timeout']:
                self.temporal.reset();self.anchor=None
            if mask_msg.encoding!='mono8': raise ValueError('road_mask must be mono8')
            if depth_msg.encoding not in ('16UC1','32FC1'): raise ValueError('Unsupported depth encoding')
            if not (mask_msg.header.frame_id==ci.header.frame_id==depth_msg.header.frame_id==di.header.frame_id):
                raise ValueError('Registered mask/depth/calibration optical frames must match')
            if (di.width,di.height)!=(depth_msg.width,depth_msg.height) or (ci.width,ci.height)!=(mask_msg.width,mask_msg.height):
                raise ValueError('Calibration dimensions do not match images')
            if di.binning_x>1 or di.binning_y>1 or ci.binning_x>1 or ci.binning_y>1 or any((di.roi.x_offset,di.roi.y_offset,ci.roi.x_offset,ci.roi.y_offset)):
                raise ValueError('ROI/binning requires calibrated effective intrinsics')
            if not np.allclose(di.r,np.eye(3).reshape(-1)) or not np.allclose(ci.r,np.eye(3).reshape(-1)):
                raise ValueError('Nonidentity rectification rotation is not supported')
            if np.any(np.abs(di.d)>1e-8): raise ValueError('Depth pinhole back projection requires rectified depth')
            if ci.distortion_model not in ('plumb_bob','rational_polynomial'): raise ValueError('Unsupported RGB distortion model')
            mask=cleanup(self.decoder.imgmsg_to_cv2(mask_msg,'passthrough'),p['morph_kernel'],p['opening'],p['closing'])
            depth=self.decoder.imgmsg_to_cv2(depth_msg,'passthrough')
            optical,labels=observations(depth,mask,di.k,p['depth_scale'],p['depth_min'],p['depth_max'],
                          p['pixel_stride'],p['alignment_mode'],ci.k,ci.d)
            base_matrix=self.matrix(p['robot_base_frame'],depth_msg.header.frame_id,depth_msg.header.stamp)
            base_points=transform(optical,base_matrix)
            planning_matrix=self.matrix(p['planning_frame'],p['robot_base_frame'],depth_msg.header.stamp)
            self.yaw=(float(np.arctan2(planning_matrix[1,0],planning_matrix[0,0]))
                      if p['grid_alignment']=='robot' else 0.0)
            robot_xy=planning_matrix[:2,3]
            if p['grid_alignment']=='planning':
                # Snap world-aligned grid origin to cell boundaries; no robot yaw in grid axes.
                offsets=np.array([p['origin_x'],p['origin_y']])
                robot_xy=np.floor((robot_xy+offsets)/p['resolution'])*p['resolution']-offsets
            ground=(base_points[:,2]>=p['ground_z_min'])&(base_points[:,2]<=p['ground_z_max'])
            points=transform(base_points[ground],planning_matrix);labels=labels[ground]
            grid,self.origin=rasterize(points,labels,p['resolution'],p['width'],p['height'],
                                      p['origin_x'],p['origin_y'],robot_xy,self.yaw)
            current_grid=grid.copy()
            if self.previous_origin is not None and not np.allclose(self.previous_origin,self.origin,atol=1e-9,rtol=0):
                self.temporal.reset()
            self.previous_origin=self.origin.copy()
            if p['temporal_frames']>1:
                if not p['temporal_fixed_frame']: raise ValueError('Temporal persistence requires fixed frame for motion detection')
                pose=self.matrix(p['temporal_fixed_frame'],p['robot_base_frame'],depth_msg.header.stamp)
                xy=pose[:2,3];yaw=float(np.arctan2(pose[1,0],pose[0,0]))
                if self.anchor is None:
                    self.anchor=(xy.copy(),yaw)
                elif np.linalg.norm(xy-self.anchor[0])>p['motion_reset_translation'] or abs(np.arctan2(np.sin(yaw-self.anchor[1]),np.cos(yaw-self.anchor[1])))>p['motion_reset_yaw']:
                    self.temporal.reset();self.anchor=(xy.copy(),yaw)
            grid=self.temporal.apply(grid)
            completion_stats={}
            if p['offline_completion']:
                grid,completion_stats=complete_unknown(grid,p['resolution'],self.origin,self.yaw,planning_matrix,
                    p['prior_x_min'],p['prior_x_max'],p['prior_half_width'],
                    p['completion_kernel'],p['completion_road_ratio'])
            if self.get_clock().now().nanoseconds*1e-9-min(stamps)>p['max_data_age']:
                raise ValueError('PROJECTION_EXPIRED_DURING_PROCESSING')
            self.publish(grid,points[labels==1],depth_msg.header.stamp,current_grid)
            self.last_success=time.monotonic();self.last_stamp=stamps[1];self.processed+=1
            bins=np.arange(-1.,2.1,.1)
            self.status(ok=True,completion=completion_stats,processed=self.processed,alignment_verified=p['alignment_verified'],
                        mock_extrinsics_configured=bool(self.mock),test_mode=p['test_mode'],
                        free_cells=int((grid==0).sum()),nonroad_cells=int((grid==100).sum()),
                        unknown_cells=int((grid==-1).sum()),ground_road_points=int((labels==1).sum()),
                        processing_latency_s=time.monotonic()-processing_started,
                        z_histogram_edges=bins.tolist(),z_histogram_counts=np.histogram(base_points[:,2],bins)[0].tolist())
        except Exception as exc:
            self.fail(str(exc))
def main(args=None):
    rclpy.init(args=args);node=None
    executor=MultiThreadedExecutor(num_threads=2)
    try:
        node=Projection();executor.add_node(node);executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        executor.shutdown()
        if node: node.destroy_node()
        rclpy.try_shutdown()
