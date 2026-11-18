"""Fuse one acquisition-bounded snapshot in odom; never write map/GNSS coordinates."""
import math
import numpy as np


def fuse_grid(center, resolution, half_width, lidar=None, camera=None):
    """lidar=(xy origin, [(angle,range)], max range); camera=(grid,origin,resolution).

    NaN rays grant no evidence. Infinite valid rays clear to range_max. Positive
    obstacle cells are applied after all ray clearing; camera combines by max.
    """
    size = int(math.ceil(2*half_width/resolution))
    origin = np.floor((np.asarray(center)-half_width)/resolution)*resolution
    grid = np.full((size,size), -1, np.int16)
    if lidar is not None:
        sensor, rays, max_range = lidar
        # Vectorize bounded ray batches. Hundreds of tiny NumPy calls release the
        # GIL repeatedly and cause large latency under ROS callback contention.
        ray_array=np.asarray(rays,dtype=float).reshape(-1,2)
        ray_array=ray_array[np.isfinite(ray_array[:,0]) & ~np.isnan(ray_array[:,1]) & (ray_array[:,1]>0)]
        reach=math.hypot(size*resolution,size*resolution)+math.dist(sensor,center)
        occupied=[]
        max_steps=max(1,int(math.ceil(min(max_range,reach)/(resolution*.5))))+1
        batch_size=max(1,1_000_000//max_steps)
        for batch in np.array_split(ray_array,max(1,math.ceil(len(ray_array)/batch_size))):
            if not len(batch):continue
            lengths=np.minimum(batch[:,1],min(max_range,reach))
            steps=max(1,int(math.ceil(float(lengths.max())/(resolution*.5))))
            distances=lengths[:,None]*np.linspace(0.,1.,steps+1)[None,:]
            x=np.floor((sensor[0]+np.cos(batch[:,0,None])*distances-origin[0])/resolution).astype(int)
            y=np.floor((sensor[1]+np.sin(batch[:,0,None])*distances-origin[1])/resolution).astype(int)
            valid=(x>=0)&(y>=0)&(x<size)&(y<size)
            grid[y[valid],x[valid]]=0
            hits=np.isfinite(batch[:,1])&(batch[:,1]<max_range)&(batch[:,1]<=reach)&valid[:,-1]
            occupied.extend(zip(x[hits,-1],y[hits,-1]))
        if occupied:
            ids=np.asarray(occupied)
            grid[ids[:,1],ids[:,0]]=100
    if camera is not None:
        values, offset, scale = camera
        rows,cols=np.indices(grid.shape)
        x=np.floor((origin[0]+(cols+.5)*resolution-offset[0])/scale).astype(int)
        y=np.floor((origin[1]+(rows+.5)*resolution-offset[1])/scale).astype(int)
        valid=(x>=0)&(y>=0)&(x<values.shape[1])&(y<values.shape[0])
        grid[valid]=np.maximum(grid[valid],values[y[valid],x[valid]])
    return grid, origin
