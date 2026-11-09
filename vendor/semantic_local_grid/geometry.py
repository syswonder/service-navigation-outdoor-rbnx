"""Pure geometry. Optical axes: +X right, +Y down, +Z forward.
No inferred camera mounting; every rigid transform is supplied explicitly.
"""
import numpy as np
import cv2

def rigid_matrix(translation, quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-9:
        raise ValueError('Invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    t=np.eye(4); t[:3,:3]=r; t[:3,3]=translation
    if not np.isfinite(t).all(): raise ValueError('Nonfinite transform')
    return t

def transform(points, matrix):
    return np.asarray(points)@matrix[:3,:3].T+matrix[:3,3]

def back_project(u,v,z,k):
    k=np.asarray(k).reshape(3,3)
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:
        raise ValueError('Invalid focal length')
    return np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)

def cleanup(mask,kernel=3,opening=False,closing=False):
    """Cleanup never expands road into non-road/reserved pixels."""
    if not opening and not closing: return mask.copy()
    if kernel<1 or kernel%2==0: raise ValueError('Odd positive morphology kernel required')
    road=(mask==1).astype(np.uint8)
    k=np.ones((kernel,kernel),np.uint8)
    filtered=road
    if opening: filtered=cv2.morphologyEx(filtered,cv2.MORPH_OPEN,k)
    if closing: filtered=cv2.morphologyEx(filtered,cv2.MORPH_CLOSE,k)
    result=mask.copy()
    result[(road==1)&(filtered==0)]=255
    # Deliberately intersect with original road; closing cannot create evidence.
    return result

def observations(depth,mask,depth_k,scale,zmin,zmax,stride=1,
                 mode='registered_same_pixels',color_k=None,color_d=None):
    if stride<1 or scale<=0 or not 0<zmin<zmax: raise ValueError('Invalid depth limits/scale/stride')
    v,u=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
    z=depth[v,u].astype(float)*scale
    valid=np.isfinite(z)&(z>0)&(z>=zmin)&(z<=zmax)
    u,v,z=u[valid],v[valid],z[valid]
    points=back_project(u,v,z,depth_k)
    if mode=='registered_same_pixels':
        if mask.shape!=depth.shape: raise ValueError('Registered image dimensions differ')
        labels=mask[v,u]
    elif mode=='rectified_depth_to_distorted_color':
        if color_k is None or color_d is None: raise ValueError('Color calibration required')
        uv,_=cv2.projectPoints(points,np.zeros(3),np.zeros(3),
                              np.asarray(color_k,dtype=float).reshape(3,3),np.asarray(color_d,dtype=float))
        uv=np.rint(uv.reshape(-1,2)).astype(int)
        good=(uv[:,0]>=0)&(uv[:,0]<mask.shape[1])&(uv[:,1]>=0)&(uv[:,1]<mask.shape[0])
        labels=np.full(len(points),255,dtype=np.uint8)
        labels[good]=mask[uv[good,1],uv[good,0]]
    else: raise ValueError('Unsupported alignment mode')
    return points,labels

def rasterize(points,labels,resolution,width,height,origin_x,origin_y,robot_xy=(0,0),yaw=0):
    # Grid is yaw-aligned to the robot, even when published in odom.
    c,s=np.cos(yaw),np.sin(yaw)
    rot=np.array([[c,-s],[s,c]])
    xy=(points[:,:2]-np.asarray(robot_xy))@rot
    index=np.floor((xy-[origin_x,origin_y])/resolution).astype(int)
    good=(index[:,0]>=0)&(index[:,0]<width)&(index[:,1]>=0)&(index[:,1]<height)
    idx=index[good,1]*width+index[good,0]; labs=labels[good]
    grid=np.full(width*height,-1,dtype=np.int8)
    grid[idx[labs==1]]=0
    # Non-road wins every collision, independently of input point ordering.
    grid[idx[labs==0]]=100
    origin=np.asarray(robot_xy)+rot@np.array([origin_x,origin_y])
    return grid.reshape(height,width),origin

class ConsecutiveRoad:
    """N consecutive CURRENT observations. No filling of current UNKNOWN cells.
    Reset on motion, TF loss, time discontinuity or watchdog expiry in caller.
    """
    def __init__(self,n=1):
        if n<1: raise ValueError('temporal_frames must be >=1')
        self.n=n; self.count=None
    def reset(self): self.count=None
    def apply(self,grid):
        if self.count is None or self.count.shape!=grid.shape:
            self.count=np.zeros(grid.shape,np.int32)
        self.count=np.where(grid==0,np.minimum(self.count+1,self.n),0)
        out=grid.copy()
        out[(grid==0)&(self.count<self.n)]=-1
        return out
