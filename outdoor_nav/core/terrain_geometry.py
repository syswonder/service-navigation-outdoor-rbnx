"""Metric terrain observations and local, source-ranked obstacle arbitration."""
import math
import numpy as np


def height_keep(points, parameters):
    p = np.asarray(points, float).reshape(-1, 3)
    h = p[:, 2] - parameters['ground_z_m']
    threshold = parameters['height_margin_m'] + np.linalg.norm(p[:, :2], axis=1) * math.tan(
        math.radians(parameters['slope_angle_deg']))
    # A ceiling follows the same envelope: the minimum cannot overtake a fixed ceiling.
    return np.isfinite(p).all(axis=1) & (h > threshold) & (h <= threshold + parameters['max_obstacle_height_m'])


def forward_mask(points, max_range, half_width=None):
    p = np.asarray(points).reshape(-1, 3)
    keep = np.isfinite(p).all(axis=1) & (p[:, 0] > 0) & (np.linalg.norm(p[:, :2], axis=1) <= max_range)
    if half_width is not None:
        keep &= np.abs(p[:, 1]) <= half_width
    return keep


def stair_heading(points, parameters):
    """Estimate uphill direction from the whole flight's principal components.

    The smallest component is the slope-plane normal. Its horizontal gradient
    resolves uphill without assuming the longest component spans stair width.
    """
    p = np.asarray(points, float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    result = dict(stairs=len(p) >= parameters['min_stair_points'], direction='unknown', heading_yaw=None)
    if not result['stairs']:
        return result
    if np.percentile(p[:, 2], 75) - parameters['ground_z_m'] < parameters['min_stair_rise_m']:
        return result
    centered = p - p.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov(centered.T))
    if values[1] <= 0 or values[1] < 4 * max(0., values[0]):
        return result
    normal = vectors[:, 0]
    if abs(normal[2]) < .1:
        return result
    uphill = -normal[:2] / normal[2]
    length = np.linalg.norm(uphill)
    if length < 1e-6 or uphill[0] <= 0:
        return result
    uphill /= length
    across = np.array([-uphill[1], uphill[0]])
    if np.ptp(p[:, :2] @ across) < parameters['min_stair_width_m']:
        return result
    result.update(direction='up', heading_yaw=float(math.atan2(uphill[1], uphill[0])))
    return result


def suppress_stair_points(points, observation, state, source, parameters):
    """True only for obstacle points matching actual semantic 3-D samples.

    Caller verifies timestamps/frames. Ties preserve obstacles. No free-space
    carving of a whole stair bounding box, nor clearing of a different object.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    mask = np.zeros(len(points), bool)
    if state.get('state') != 'stairs_up' or state.get('phase') != 'climb':
        return mask
    rank = parameters['priorities']['stairs_up']
    if not observation or not observation.get('stairs') or observation.get('direction') != 'up' or rank.get(observation.get('source'), -1) <= rank[source]:
        return mask
    samples = np.asarray(observation.get('points', []), float).reshape(-1, 3)
    if not len(samples):
        return mask
    from scipy.spatial import cKDTree
    distance, _ = cKDTree(samples).query(points, k=1, distance_upper_bound=parameters['semantic_match_radius_m'])
    return np.isfinite(distance)
