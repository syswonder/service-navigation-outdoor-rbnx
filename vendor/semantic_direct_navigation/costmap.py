"""Per-process rolling costmap; no persistence or cross-session recovery."""
import numpy as np
import shapely
from .core import grid_regions
from semantic_local_grid.obstacle_geometry import fuse, inflate

def build_costmap(road, visual, lidar, center, resolution=.05, half_window=6.,
                  semantic_road=None, semantic_nonroad=None):
    center=np.asarray(center,float)
    if center.shape!=(2,) or not np.isfinite(center).all():
        raise ValueError('Invalid costmap center')
    origin=np.floor((center-half_window)/resolution)*resolution
    n=int(np.ceil(2*half_window/resolution))
    rr,cc=np.indices((n,n))
    x=origin[0]+cc.ravel()*resolution
    y=origin[1]+rr.ravel()*resolution
    cells=shapely.box(x,y,x+resolution,y+resolution)
    semantic=np.full((n,n),-1,np.int16)
    shapely.prepare(road)
    semantic.ravel()[shapely.intersects(road,cells)]=0
    obstacle=np.full_like(semantic,-1)
    for source in (visual,lidar):
        points=np.asarray(source,float).reshape(-1,2)
        if not np.isfinite(points).all(): raise ValueError('Invalid obstacle points')
        ids=np.floor((points-origin)/resolution).astype(int)
        ids=ids[((ids>=0)&(ids<n)).all(axis=1)]
        obstacle[ids[:,1],ids[:,0]]=100
    raw=fuse(semantic,obstacle)
    costs=inflate(raw,resolution,.3)
    # PIDNet is auxiliary: only add a soft preference on geometrically known
    # free cells. Unknown/physical obstacles never become traversable by class.
    if semantic_nonroad is not None and not semantic_nonroad.is_empty:
        centers=shapely.points(x+resolution/2,y+resolution/2)
        penalty=shapely.covers(semantic_nonroad,centers)
        if semantic_road is not None and not semantic_road.is_empty:
            penalty &= ~shapely.covers(semantic_road,centers)
        mask=penalty.reshape(raw.shape)&(raw==0)
        costs[mask]=np.maximum(costs[mask],40)
    costs[raw==-1]=-1  # Standard output must not label unknown as inflated free space.
    # Unknown stays unknown even inside an inflation ring. Robot radius is
    # checked by the planner, so soft inflation is not a second hard radius.
    labels=np.where((raw==0)&(costs<100),0,-1)
    decoded=grid_regions(labels,resolution,origin,include_nonroad=False)[0]
    # Rasterization may never add coverage beyond original observed geometry.
    return raw,costs,origin,resolution,decoded.intersection(road)


class VisualObstacleMemory:
    """Bounded positive-evidence hold on fixed planning-frame cells.

    Repeated control ticks never refresh a sensor frame's timestamp. New
    positives are immediate; disappearance expires after hold_s. No disk state.
    """
    def __init__(self, hold_s=.8, resolution=.05, half_window=6.):
        if not np.isfinite([hold_s,resolution,half_window]).all() or not (
                0 < hold_s <= 2. and .02 <= resolution <= .2 and 1 <= half_window <= 15):
            raise ValueError('Invalid visual obstacle memory settings')
        self.hold_s,self.resolution,self.half_window=hold_s,resolution,half_window
        self.clear()

    def clear(self):
        self.cells={}
        self.epoch=None
        self.last_stamp=-np.inf
        self.last_now=-np.inf

    def update(self, points, stamp, now, center, epoch):
        points=np.asarray(points,float).reshape(-1,2)
        center=np.asarray(center,float)
        if (center.shape!=(2,) or not np.isfinite(center).all()
                or not np.isfinite(points).all() or not np.isfinite([stamp,now]).all()):
            raise ValueError('Invalid visual obstacle memory input')
        if epoch!=self.epoch or now<self.last_now:
            self.clear()
        self.epoch,self.last_now=epoch,now
        self.cells={k:t for k,t in self.cells.items() if 0<=now-t<self.hold_s
                    and np.all(abs((np.asarray(k)+.5)*self.resolution-center)<=self.half_window)}
        if stamp>self.last_stamp and 0<=now-stamp<self.hold_s:
            current=points[(abs(points-center)<=self.half_window).all(axis=1)]
            for key in np.unique(np.floor(current/self.resolution).astype(int),axis=0):
                self.cells[tuple(key)]=stamp
            self.last_stamp=stamp
        retained=(np.asarray(list(self.cells),float).reshape(-1,2)+.5)*self.resolution
        # Keep exact current positives too: voxel centres must never replace
        # a closer measured obstacle in the swept collision checks.
        current=points if 0<=now-stamp<self.hold_s else np.empty((0,2))
        return np.vstack((current,retained))
