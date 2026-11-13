"""Model-neutral ROS adapter, adapted from vendored Apache-2.0 pidnet_node.py."""
import time,json
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_sanity_checker.images import ImageDecoder
from .segmentation import create_backend, predict_labels, UNKNOWN
from semantic_local_grid.ros_utils import image_message,seconds

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('semantic_segmentation')
        defaults={'rgb_topic':'/camera_gemini330/color/image_raw','mask_topic':'/semantic/road_mask',
                  'labels_topic':'/outdoor/vision/semantic_labels',
                  'confidence_topic':'/outdoor/vision/semantic_confidence',
                  'status_topic':'/semantic/segmentation_status',
                  'overlay_topic':'/semantic/debug_overlay','config_json':'{}','max_image_age':1.0}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        self.decoder=ImageDecoder()
        self.config=json.loads(self.p['config_json'])
        self.model=create_backend(self.config)
        self.pub=self.create_publisher(Image,self.p['mask_topic'],5)
        self.labels=self.create_publisher(Image,self.p['labels_topic'],5)
        self.confidence=self.create_publisher(Image,self.p['confidence_topic'],5)
        self.debug=self.create_publisher(Image,self.p['overlay_topic'],2)
        self.status=self.create_publisher(String,self.p['status_topic'],5)
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
            labels,confidence,mask=predict_labels(self.model,rgb,self.config)
            elapsed=time.monotonic()-start
            if self.get_clock().now().nanoseconds*1e-9-seconds(msg.header.stamp)>self.p['max_image_age']:
                raise ValueError('Inference output is stale; discarded')
            self.pub.publish(image_message(mask,msg.header,'mono8'))
            self.labels.publish(image_message(labels,msg.header,'mono16'))
            self.confidence.publish(image_message(confidence,msg.header,'32FC1'))
            debug = rgb.copy()
            road = mask == 1
            debug[road] = (debug[road].astype(float)*.55 + np.array([0,255,0])*.45).astype(np.uint8)
            self.debug.publish(image_message(debug,msg.header,'rgb8'))
            self.frames+=1
            self.status.publish(String(data=json.dumps({'ok':True,'backend':self.config['backend'],
                      'classes':self.config['classes'],'unknown_id':UNKNOWN,
                      'road_classes':self.config['road_classes'],
                      'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},
                      'frame_id':msg.header.frame_id,
                      'frames':self.frames,'inference_seconds':elapsed,'road_pixels':int((mask==1).sum())})))
        except Exception as exc:
            self.status.publish(String(data=json.dumps({'ok':False,'error':str(exc)})))
            self.get_logger().error(str(exc))
def main(args=None):
    rclpy.init(args=args);node=None
    try: node=SegmentationNode();rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.destroy_node()
        rclpy.try_shutdown()
