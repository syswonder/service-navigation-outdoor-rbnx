import numpy as np
from sensor_msgs.msg import Image,PointCloud2,PointField
from .geometry import rigid_matrix

def matrix_from_tf(t):
    v=t.transform.translation;q=t.transform.rotation
    return rigid_matrix([v.x,v.y,v.z],[q.x,q.y,q.z,q.w])

def image_message(array,header,encoding):
    msg=Image();msg.header=header;msg.height=array.shape[0];msg.width=array.shape[1]
    msg.encoding=encoding;msg.is_bigendian=0;msg.step=array.strides[0]
    msg.data=np.ascontiguousarray(array).tobytes()
    return msg

def cloud_message(points,header):
    data=np.asarray(points,dtype='<f4').reshape(-1,3)
    msg=PointCloud2();msg.header=header;msg.height=1;msg.width=len(data)
    msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
    msg.is_bigendian=False;msg.point_step=12;msg.row_step=len(data)*12
    msg.is_dense=True;msg.data=data.tobytes()
    return msg

def seconds(stamp): return stamp.sec+stamp.nanosec*1e-9
