"""Stride/endian-aware ROS Image decoder without cv_bridge's native conversion ABI."""
import numpy as np

class ImageDecoder:
    def imgmsg_to_cv2(self,msg,desired_encoding='passthrough'):
        formats={'rgb8':('u1',3),'bgr8':('u1',3),'mono8':('u1',1),
                 '8UC1':('u1',1),'16UC1':('u2',1),'mono16':('u2',1),'32FC1':('f4',1)}
        if msg.encoding not in formats:
            raise ValueError('Unsupported encoding '+msg.encoding)
        code,channels=formats[msg.encoding]
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+code)
        if msg.step<msg.width*channels*dtype.itemsize or len(msg.data)<msg.height*msg.step:
            raise ValueError('Invalid Image step/data length')
        shape=(msg.height,msg.width,channels) if channels>1 else (msg.height,msg.width)
        strides=(msg.step,channels*dtype.itemsize,dtype.itemsize) if channels>1 else (msg.step,dtype.itemsize)
        image=np.ndarray(shape,dtype=dtype,buffer=msg.data,strides=strides)
        image=image.astype(dtype.newbyteorder('='),copy=True)
        if desired_encoding=='bgr8':
            if msg.encoding=='rgb8': return image[:,:,::-1].copy()
            if msg.encoding=='bgr8': return image
            if msg.encoding in ('mono8','8UC1'): return np.repeat(image[:,:,None],3,axis=2)
            raise ValueError('Cannot interpret '+msg.encoding+' as color')
        if desired_encoding!='passthrough': raise ValueError('Unsupported target encoding')
        return image
