"""Independent geometric ground/obstacles; semantic labels never erase returns.

Empty scans and invalid ground fits are invalid evidence, not free space.
Only directly observed ground, with bounded local interpolation, grants space.
"""
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import Point
from .core import grid_regions


def rolling_blind_zone(pose, forward_m, half_length, safety_radius):
    """Explicit moving coverage prior; never removes measured hazard points.

    forward_m is measured from the vehicle front, not from its centre.
    """
    from shapely.geometry import box
    from shapely.affinity import rotate, translate
    pose = np.asarray(pose, float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError('Invalid blind-zone pose')
    if not np.isfinite([forward_m,half_length,safety_radius]).all() or not 0 < forward_m <= .8 or min(half_length,safety_radius)<=0:
        raise ValueError('Blind-zone prior must be within the confirmed 0.8 m')
    area=box(-safety_radius-.01,-safety_radius-.02,half_length+forward_m,safety_radius+.02)
    return translate(rotate(area,pose[2],origin=(0,0),use_radians=True),xoff=pose[0],yoff=pose[1])


class GroundCoverage:
    """Bounded observed-ground history plus an explicit, fixed startup prior.

    This supplies ground coverage only; callers must still check every positive
    obstacle, source freshness and the complete stopping footprint.
    """
    def __init__(self, startup_clearance_m=0., history_s=20.):
        if not np.isfinite(startup_clearance_m) or not 0 <= startup_clearance_m <= 1.:
            raise ValueError('Startup clearance must be within the confirmed 0..1 m')
        self.forward = startup_clearance_m
        self.history_s = history_s
        self.anchor = None
        self.epoch = None
        self.prior = None
        self.batches = []
        self.cells = {}
        self.last_stamp = -np.inf
        self.version = 0
        self.invalidated = False

    def bind(self, pose, epoch, radius):
        from shapely.geometry import box
        from shapely.affinity import rotate, translate
        if self.anchor is not None:
            if epoch != self.epoch:
                self.batches = []
                self.cells = {}
                self.prior = None
                self.invalidated = True
                self.epoch = epoch
                self.last_stamp = -np.inf
                self.version += 1
            return
        self.anchor = np.asarray(pose, float).copy()
        self.epoch = epoch
        if self.forward:
            # Front edge is exactly the confirmed distance, not distance+radius.
            shape = box(-radius-.01, -radius-.02, self.forward, radius+.02)
            self.prior = translate(rotate(shape, pose[2], origin=(0, 0), use_radians=True),
                                   xoff=pose[0], yoff=pose[1])

    def update(self, points, stamp, now, pose, epoch, radius):
        self.bind(pose, epoch, radius)
        current = np.asarray(points, float).reshape(-1, 2)
        old_count = len(self.batches)
        self.batches = [(t, p) for t, p in self.batches if 0 <= now-t <= self.history_s]
        if len(self.batches) != old_count:
            self.version += 1
        if stamp-self.last_stamp >= .25 and 0 <= now-stamp <= .8:
            # Only retain the near-body band that is about to enter the camera
            # blind zone. Far ground remains current-frame evidence, avoiding
            # accumulation of a costly 10 m map for this narrow purpose.
            p = current[np.linalg.norm(current-np.asarray(pose)[:2], axis=1) <= 1.8]
            if len(p):
                _, ids = np.unique(np.floor(p/.06), axis=0, return_index=True)
                self.batches.append((stamp, p[ids]))
                self.batches = self.batches[-81:]
                for key, point in zip(np.floor(p[ids]/.06).astype(int), p[ids]):
                    self.cells[tuple(key)] = (stamp, point)
            self.last_stamp = stamp
            self.version += 1
        # The fixed startup area is permanently retired after leaving it.
        if self.prior is not None and np.linalg.norm(np.asarray(pose)[:2]-self.anchor[:2]) > self.forward+2*radius:
            self.prior = None
            self.version += 1
        center = np.asarray(pose)[:2]
        self.cells = {k:(t,p) for k,(t,p) in self.cells.items()
                      if 0 <= now-t <= self.history_s and (p[0]-center[0])**2+(p[1]-center[1])**2 <= 1.8**2}
        remembered = np.array([p for _,p in self.cells.values()]).reshape(-1,2)
        points = np.vstack((current, remembered))
        points = points[(abs(points-center) <= 5.).all(axis=1)]
        if len(points):
            _, ids = np.unique(np.floor(points/.06), axis=0, return_index=True)
            points = points[ids]
        return points, self.prior


def current_body_region(pose, safety_radius):
    # Numerical arc/interpolation padding only. This is occupied body space,
    # never the old one-metre forward traversability assumption.
    return Point(np.asarray(pose)[:2]).buffer(safety_radius+.001, quad_segs=64)


def _independent_observation(points, *, expected_ground_z=-.18, ground_z_tolerance=.05,
                          min_height=.10, max_height=1.5, min_ground=40):
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    # Zero ranges must not be transformed into apparent ground or obstacles.
    points = points[np.linalg.norm(points, axis=1) > .05]
    roi = ((points[:, 0] > -.8) & (points[:, 0] < 5.) & (abs(points[:, 1]) < 3.))
    points = points[roi]
    empty = np.empty((0, 3))
    status = dict(valid=False, source='INDEPENDENT_GEOMETRY', raw_points=len(points))
    if (not np.isfinite([expected_ground_z, ground_z_tolerance, min_height]).all()
            or expected_ground_z >= 0 or not 0 < ground_z_tolerance <= .10
            or not 0 < min_height < max_height):
        raise ValueError('Invalid geometric ground calibration/threshold')
    seeds = points[(points[:, 0] > .15) & (abs(points[:, 1]) < 2.5)
                   & (abs(points[:, 2]-expected_ground_z) < .25)]
    if len(seeds) < min_ground:
        return empty, empty, dict(status, reason='INSUFFICIENT_GEOMETRIC_GROUND')
    # Voxelize BEFORE fitting: image density must not drown out a second surface.
    _, ids = np.unique(np.floor(seeds/.035), axis=0, return_index=True)
    fit = seeds[ids]
    fit = fit[::max(1, int(np.ceil(len(fit)/1600)))]
    if len(fit) < 30:
        return empty, empty, dict(status, reason='INSUFFICIENT_SPATIAL_GROUND')
    a = np.column_stack((fit[:, :2], np.ones(len(fit))))
    best = None
    rng = np.random.default_rng(2710)
    for _ in range(70):
        ids = rng.choice(len(fit), 3, replace=False)
        try:
            model = np.linalg.solve(a[ids], fit[ids, 2])
        except np.linalg.LinAlgError:
            continue
        if (np.linalg.norm(model[:2]) > np.tan(np.deg2rad(12.))
                or abs(model[2]-expected_ground_z) > ground_z_tolerance):
            continue
        keep = abs(a@model-fit[:, 2]) < .018
        n = int(keep.sum())
        if best is None or n > best[0]:
            best = n, keep
    if best is None or best[0] < max(30, len(fit)*.35):
        return empty, empty, dict(status, reason='GEOMETRIC_GROUND_UNCERTAIN')
    support = fit[best[1]]
    if np.ptp(support[:, 0]) < .5 or np.ptp(support[:, 1]) < .25:
        return empty, empty, dict(status, reason='GEOMETRIC_GROUND_TOO_NARROW')
    model = np.linalg.lstsq(a[best[1]], support[:, 2], rcond=None)[0]
    if (abs(model[2]-expected_ground_z) > ground_z_tolerance
            or np.linalg.norm(model[:2]) > np.tan(np.deg2rad(12.))):
        return empty, empty, dict(status, reason='GEOMETRIC_GROUND_CALIBRATION_MISMATCH')
    residual = points[:, 2] - np.column_stack((points[:, :2], np.ones(len(points))))@model
    error = support[:, 2] - a[best[1]]@model
    noise = float(1.4826*np.median(abs(error-np.median(error))))
    threshold = min_height
    ground = points[(abs(residual) <= .018) & (points[:, 0] > -.8)
                    & (abs(points[:, 1]) < 2.5)]
    # Both raised returns and measured depressions are hard hazards. Sparse
    # LiDAR positives are retained; no semantic class or cluster veto applies.
    hazard = ((residual > threshold) & (residual < max_height)) | (residual < -.10)
    obstacles = points[hazard]
    return ground, obstacles, dict(status, valid=True, reason='OK',
        ground_points=len(ground), obstacle_points=len(obstacles),
        plane_abc=model.tolist(), noise_m=noise, height_threshold_m=threshold)


def geometric_observation(points, *, expected_ground_z=-.18, ground_z_tolerance=.05,
                          min_height=.10, max_height=1.5, min_ground=40,
                          prior_plane=None):
    """Fit own ground first; an explicitly validated geometric prior may back up
    a scan without ground returns. Caller must enforce prior freshness/frame.
    A prior supplies height, never free-space or an absent obstacle return.
    """
    ground, obstacles, status = _independent_observation(points,
        expected_ground_z=expected_ground_z, ground_z_tolerance=ground_z_tolerance,
        min_height=min_height, max_height=max_height, min_ground=min_ground)
    if status['valid']:
        status['plane_source'] = 'CURRENT_SCAN_GEOMETRY'
        return ground, obstacles, status
    if prior_plane is None:
        return ground, obstacles, status
    model = np.asarray(prior_plane, float)
    if (model.shape != (3,) or not np.isfinite(model).all()
            or abs(model[2]-expected_ground_z) > ground_z_tolerance
            or np.linalg.norm(model[:2]) > np.tan(np.deg2rad(12.))):
        return ground, obstacles, dict(status, reason='INVALID_GEOMETRIC_PRIOR')
    raw = np.asarray(points, float).reshape(-1, 3)
    raw = raw[np.isfinite(raw).all(axis=1) & (np.linalg.norm(raw, axis=1) > .05)]
    if len(raw) < 10:
        return ground, obstacles, dict(status, reason='EMPTY_OR_INSUFFICIENT_LIDAR_SCAN')
    raw = raw[(raw[:,0]>-.8)&(raw[:,0]<5)&(abs(raw[:,1])<3)]
    residual=raw[:,2]-np.column_stack((raw[:,:2],np.ones(len(raw))))@model
    ground=raw[(abs(residual)<=.018)&(raw[:,0]>-.8)&(abs(raw[:,1])<2.5)]
    obstacles=raw[((residual>min_height)&(residual<max_height))|(residual<-.10)]
    return ground,obstacles,dict(status, valid=True, reason='OK',
        plane_source='FRESH_DEPTH_GEOMETRY',plane_abc=model.tolist(),
        ground_points=len(ground),obstacle_points=len(obstacles),height_threshold_m=min_height)


def observed_ground_region(points, center, *, resolution=.10, support_distance=.12,
                           half_window=5.):
    """Union small measured cells; no ray clearing, convex hull, or route fill.

    Every cell corner must be near measured ground. Large gaps/drop-offs remain
    holes. This is bounded ground interpolation, not a guarantee about grass.
    """
    points = np.asarray(points, float).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    center = np.asarray(center, float)
    if not 0 < resolution <= .15 or not 0 < support_distance <= .15:
        raise ValueError('Unbounded ground interpolation')
    origin = np.floor((center-half_window)/resolution)*resolution
    n = int(np.ceil(2*half_window/resolution))
    grid = np.full((n, n), -1, np.int8)
    if len(points):
        points = points[(np.abs(points-center) <= half_window+.2).all(axis=1)]
    if len(points):
        _, ids = np.unique(np.floor(points/.035), axis=0, return_index=True)
        tree = cKDTree(points[ids])
        rr, cc = np.indices(grid.shape)
        xy = origin + np.column_stack((cc.ravel()+.5, rr.ravel()+.5))*resolution
        # Cells outside the support bounding box cannot pass the corner test.
        # Avoid four nearest-neighbour queries over the entire unknown window.
        candidates = ((xy >= points.min(axis=0)-support_distance-resolution).all(axis=1)
                      & (xy <= points.max(axis=0)+support_distance+resolution).all(axis=1))
        selected = xy[candidates]
        valid = np.ones(len(selected), bool)
        for corner in ((-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)):
            valid &= tree.query(selected+np.asarray(corner)*resolution)[0] <= support_distance
        grid.ravel()[np.flatnonzero(candidates)[valid]] = 0
    return grid_regions(grid, resolution, origin, include_nonroad=False)[0]
