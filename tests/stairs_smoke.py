"""Real Nav2 with virtual inputs on localhost domain 188, never robot domains 0/80."""
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time

os.environ.update(ROS_DOMAIN_ID='188', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String, UInt32
from tf2_msgs.msg import TFMessage
from outdoor_nav.manager import RuntimeManager

ROOT=Path(__file__).resolve().parents[1]
ORIGIN={'latitude':39.9042,'longitude':116.4074}
OUTPUT='/field_smoke/command'


class Sensors(Node):
    def __init__(self):
        super().__init__('field_smoke_virtual_sensors')
        self.x=self.v=self.at=0.
        self.enabled=False
        self.gait=0x3002
        self.pending=None
        self.ignore_gait=False
        self.requests=[]
        self.moved_without_confirmation=False
        self.odom=self.create_publisher(Odometry,'/field_smoke/odom',10)
        self.geo=self.create_publisher(String,'/field_smoke/localization',10)
        self.chassis=self.create_publisher(String,'/field_smoke/chassis',10)
        self.tf=self.create_publisher(TFMessage,'/field_smoke/tf',10)
        self.cloud=self.create_publisher(PointCloud2,'/field_smoke/cloud',10)
        self.create_subscription(Twist,OUTPUT,self.command,10)
        self.create_subscription(UInt32,'/field_smoke/gait',self.switch,10)
        self.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
        self.create_timer(.05,self.tick)

    def command(self,msg):
        self.v,self.at=msg.linear.x,time.monotonic()
        if self.pending and abs(self.v)>.03:
            self.moved_without_confirmation=True

    def switch(self,msg):
        self.requests.append({'target':msg.data,'speed':self.v})
        if not self.ignore_gait and self.pending is None:
            self.pending=(msg.data,time.monotonic()+.4)

    def tick(self):
        if not self.enabled:
            return
        now=time.monotonic()
        if now-self.at>.4:self.v=0.
        if self.pending and now>=self.pending[1]:
            self.gait=self.pending[0];self.pending=None
        self.x+=self.v*.05
        stamp=self.get_clock().now().to_msg()
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.pose.pose.position.x=self.x;odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.x=self.v
        self.odom.publish(odom)
        transforms=[]
        for parent,child,x in (('map','odom',0.),('odom','base_link',self.x)):
            tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id=parent;tr.child_frame_id=child
            tr.transform.translation.x=x;tr.transform.rotation.w=1.;transforms.append(tr)
        self.tf.publish(TFMessage(transforms=transforms))
        self.geo.publish(String(data=json.dumps({'ready':True,'grade':'fixed','extrinsics':{'ready':True},
            'heading':{'ready':True},'coordinate_reference':{'frame_id':'map','axes':'east_north',
                'origin_wgs84':ORIGIN,'map_offset_xy_m':[0.,0.],'antenna_xy_m':[0.,0.],
                'map_id':'synthetic-field','version':'f'*16}})))
        self.chassis.publish(String(data=json.dumps({'ready':True,'state':17,'usage_mode':1,
            'gait':self.gait,'motion_feedback_age_s':0.,'usage_mode_age_s':0.,'gait_output_enabled':True})))
        rays=[(7.5*math.cos(i*math.pi/180),7.5*math.sin(i*math.pi/180),0.) for i in range(360)]
        # A visible synthetic step lies on the reference; stairs must not invoke obstacle detours.
        rays.extend((.8-self.x,y/20,0.) for y in range(-4,5))
        self.cloud.publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='base_link'),self.fields,rays))
        assert self.count_publishers('/NAV_CMD')==0 and self.count_publishers('/GAIT')==0


def wait(check,seconds):
    deadline=time.monotonic()+seconds
    while True:
        result=check()
        if result:return result
        if time.monotonic()>=deadline:raise TimeoutError('Virtual workflow timed out')
        time.sleep(.05)


rclpy.init();sensors=Sensors();executor=SingleThreadedExecutor();executor.add_node(sensors)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
manager=RuntimeManager()
report={'robot_motion':False,'domain':188}
try:
    folder=tempfile.mkdtemp(prefix='stairs-nav2-smoke-')
    config={'params_file':str(ROOT/'config/nav2_params.example.yml'),'run_dir':folder,'http_port':19986,
        'velocity_output_topic':OUTPUT,'gait_switching_enabled':True,'gait_output_topic':'/field_smoke/gait',
        'global_grid':{'origin':[-10.,-10.],'width':20.,'height':20.,'resolution':.1,'rolling':True},
        'topic_remap':{k:'/field_smoke/'+v for k,v in {'odom':'odom','cloud':'cloud','scan':'scan',
            'localization':'localization','chassis':'chassis','tf':'tf','tf_static':'tf_static'}.items()},
        'policy':{'motion_enabled':True,'calibrated':True,'antenna_offset_confirmed':True,
            'footprint_confirmed':True,'cloud_frame_confirmed':True,'odometry_confirmed':True,
            'localization_mode':'external_odom','heading_source':'configured','require_chassis_ready':True,
            'gait_switch_timeout_s':4.,'fixed_arrival_m':.4,'float_arrival_m':.4}}
    manager.start(config)
    assert not manager.request('/api/status')['ready']
    report['blank_map_startup']=True
    manager.active(True)
    sensors.enabled=True
    wait(lambda:manager.request('/api/status')['ready'],10)
    request={'frame_id':'map','map_id':'synthetic-field','version':'f'*16,'origin_wgs84':ORIGIN,
        'reference_path':[{'x':0.,'y':0.},{'x':3.,'y':0.}],'goal':{'x':3.,'y':0.},'request_id':'d'*32,
        'intervals':[{'start_m':0.,'end_m':1.2,'gait':'stairs','gait_code':0x1003},
                     {'start_m':1.2,'end_m':3.,'gait':'flat','gait_code':0x1001}]}
    manager.request('/api/tasks',request)
    modes=set()
    def arrived():
        status=manager.request('/api/status');modes.add(status['local_avoidance_enabled'])
        task=status['task']
        if task and task['state'] in ('failed','canceled'):raise AssertionError(task)
        return task if task and task['state']=='succeeded' else None
    first=wait(arrived,45)
    assert sensors.requests and all(abs(r['speed'])<=.03 for r in sensors.requests),sensors.requests
    assert not sensors.moved_without_confirmation
    assert modes=={True,False},modes
    assert {r['target'] for r in sensors.requests}=={0x3002,0x3003}
    report.update(arrival=first['state'],avoidance_paused_and_restored=True,stopped_before_switch=True)
    sensors.ignore_gait=True
    start=sensors.x
    request.update(request_id='e'*32,reference_path=[{'x':start,'y':0.},{'x':start+3.,'y':0.}],
        goal={'x':start+3.,'y':0.},intervals=[{'start_m':0.,'end_m':3.,'gait':'stairs','gait_code':0x1003}])
    manager.request('/api/tasks',request)
    def failed():
        task=manager.request('/api/status')['task']
        return task if task and task['state'] in ('failed','succeeded') else None
    failure=wait(failed,12)
    assert failure['state']=='failed' and 'gait_switch_timeout' in failure['reason'],failure
    assert abs(sensors.x-start)<.05 and abs(sensors.v)<.03
    report.update(unconfirmed_switch_failed_stopped=True,passed=True)
    manager.shutdown()
except Exception:
    print('virtual x/speed/gait:',sensors.x,sensors.v,sensors.gait,'switches:',sensors.requests,flush=True)
    print('status:',manager.request('/api/status'),flush=True)
    if manager.runtime:
        log=Path(manager.runtime['run_dir'])/'worker.log'
        if log.exists():print(log.read_text()[-6000:],flush=True)
    raise
finally:
    manager.shutdown();executor.shutdown();thread.join(timeout=2);sensors.destroy_node();rclpy.shutdown()
print(json.dumps(report),flush=True)
