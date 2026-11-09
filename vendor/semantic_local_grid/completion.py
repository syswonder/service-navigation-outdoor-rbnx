"""Explicit offline prior: inferred free cells are not sensor observations."""
import numpy as np
import cv2

def complete_unknown(grid, resolution, origin, grid_yaw, planning_from_base,
                     x_min=-0.3, x_max=1.3, half_width=0.6,
                     kernel=9, road_ratio=0.55):
    if kernel < 3 or kernel % 2 != 1 or not 0 < road_ratio <= 1:
        raise ValueError('kernel must be odd >=3 and road_ratio in (0,1]')
    if not x_min < x_max or half_width <= 0:
        raise ValueError('Invalid bounded near-field region')
    original=np.asarray(grid);result=original.copy()
    # One pass, using only measured road; invented near-field free cannot seed further growth.
    road=(original==0).astype(np.float32)
    ratio=cv2.boxFilter(road,-1,(kernel,kernel),normalize=True,borderType=cv2.BORDER_CONSTANT)
    nonroad=cv2.dilate((original==100).astype(np.uint8),np.ones((kernel,kernel),np.uint8))>0
    fill=(original==-1)&(ratio>=road_ratio)&~nonroad
    result[fill]=0
    yy,xx=np.indices(original.shape)
    local=np.stack(((xx+.5)*resolution,(yy+.5)*resolution),axis=-1)
    c,s=np.cos(grid_yaw),np.sin(grid_yaw)
    world=local@np.array([[c,s],[-s,c]])+np.asarray(origin)
    pose=planning_from_base
    # Robot-frame rectangle, independent of odom position/yaw.
    base=(world-pose[:2,3])@pose[:2,:2]
    near=(base[...,0]>=x_min)&(base[...,0]<=x_max)&(np.abs(base[...,1])<=half_width)
    prior=(original==-1)&near
    result[prior]=0
    return result, {'inferred_near_cells':int(prior.sum()),
                    'inferred_neighborhood_cells':int((fill&~prior).sum()),
                    'observed_road_cells':int((original==0).sum()),
                    'inferred_total_cells':int(((original==-1)&(result==0)).sum())}
