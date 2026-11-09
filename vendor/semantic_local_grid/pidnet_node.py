import os,time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_sanity_checker.images import ImageDecoder
from .inference import Segmenter,mock_mask,overlay
from .ros_utils import image_message,seconds

class PIDNetNode(Node):
    def __init__(self):
        super().__init__('pidnet_node')
        defaults={'rgb_topic':'/camera_gemini330/color/image_raw','mask_topic':'/semantic/road_mask',
                  'overlay_topic':'/semantic/debug_overlay','pidnet_weights':'','mode':'pidnet',
                  'test_mode':False,'device':'auto','architecture':'pidnet-m','confidence_threshold':0.7,
                  'max_image_age':1.0,'cpu_threads':4,'traversable_train_ids':[0,1]}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        if self.p['mode'] not in ('pidnet','mock'): raise ValueError('mode must be pidnet or mock')
        if self.p['mode']=='mock' and not self.p['test_mode']: raise ValueError('Mock segmentation requires test_mode')
        self.decoder=ImageDecoder()
        self.model=None
        if self.p['mode']=='pidnet':
            self.model=Segmenter(self.p['pidnet_weights'],self.p['device'],self.p['architecture'],
                                 self.p['confidence_threshold'],self.p['cpu_threads'],self.p['traversable_train_ids'])
            self.model.predict(np.zeros((480,640,3),np.uint8))
            self.get_logger().info(json.dumps(self.model.metadata))
        else: self.get_logger().warning('MOCK segmentation: synthetic corridor, NOT PIDNet evidence')
        self.pub=self.create_publisher(Image,self.p['mask_topic'],5)
        self.debug=self.create_publisher(Image,self.p['overlay_topic'],2)
        self.status=self.create_publisher(String,'/semantic/pidnet_status',5)
        self.sub=self.create_subscription(Image,self.p['rgb_topic'],self.receive,
                                         rclpy.qos.QoSProfile(depth=1,reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.frames=0
    def receive(self,msg):
        try:
            age=self.get_clock().now().nanoseconds*1e-9-seconds(msg.header.stamp)
            if not msg.header.frame_id or seconds(msg.header.stamp)<=0 or age>self.p['max_image_age'] or age<-.1:
                raise ValueError('RGB missing header or stale/future timestamp')
            rgb=self.decoder.imgmsg_to_cv2(msg,'bgr8')[:,:,::-1].copy()
            start=time.monotonic()
            mask=self.model.predict(rgb)[0] if self.model else mock_mask(rgb)
            elapsed=time.monotonic()-start
            if self.get_clock().now().nanoseconds*1e-9-seconds(msg.header.stamp)>self.p['max_image_age']:
                raise ValueError('Inference output is stale; discarded')
            self.pub.publish(image_message(mask,msg.header,'mono8'))
            self.debug.publish(image_message(overlay(rgb,mask),msg.header,'rgb8'))
            self.frames+=1
            self.status.publish(String(data=json.dumps({'ok':True,'mode':self.p['mode'],
                      'frames':self.frames,'inference_seconds':elapsed,'road_pixels':int((mask==1).sum())})))
        except Exception as exc:
            self.status.publish(String(data=json.dumps({'ok':False,'error':str(exc)})))
            self.get_logger().error(str(exc))
def main(args=None):
    rclpy.init(args=args);node=None
    try: node=PIDNetNode();rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.destroy_node()
        rclpy.try_shutdown()
