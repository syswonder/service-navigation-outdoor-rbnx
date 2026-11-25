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


def nearest_stair_flight(points, parameters):
    """Fit uphill as before, using the nearest run when a landing separates the samples."""
    fit = stair_heading(points, parameters)
    if not fit['stairs']:
        return fit, points
    heading = fit['heading_yaw']
    direction = [math.cos(heading), math.sin(heading)] if heading is not None else [1., 0.]
    along = points[:, :2] @ direction
    ordered = np.sort(along)
    gaps = np.flatnonzero(np.diff(ordered) >= parameters['stair_landing_gap_m'])
    if len(gaps):
        points = points[along <= ordered[gaps[0]]]
        fit = stair_heading(points, parameters)
        fit['stairs'] = True  # A partial foreground fit cannot certify a clear path.
    heading = fit['heading_yaw']
    if heading is not None:
        normal = np.array([math.cos(heading), math.sin(heading)])
        lateral = np.array([-normal[1], normal[0]])
        xy = points[:, :2]
        along = xy @ normal
        center = np.median(xy @ lateral)*lateral
        fit['flight_bounds'] = dict(front_xy=(center+along.min()*normal).tolist(),
                                    end_xy=(center+along.max()*normal).tolist())
    return fit, points


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
    eligible = np.ones(len(points), bool)
    flight = state.get('stair_flight')
    if flight is not None:
        heading = flight['heading_yaw']
        normal = np.array([math.cos(heading), math.sin(heading)])
        lateral = np.array([-normal[1], normal[0]])
        front = np.asarray(flight['front_xy'])
        margin = min(parameters['semantic_match_radius_m'], parameters['stair_landing_gap_m']/3)

        def in_flight(values):
            offsets = values[:, :2]-front
            along = offsets @ normal
            return ((along >= -margin) & (along <= flight['length_m']+margin)
                    & (abs(offsets @ lateral) <= parameters['roi_half_width_m']))

        eligible = in_flight(points)
        samples = samples[in_flight(samples)]
    if not len(samples):
        return mask
    from scipy.spatial import cKDTree
    distance, _ = cKDTree(samples).query(points[eligible], k=1, distance_upper_bound=parameters['semantic_match_radius_m'])
    mask[eligible] = np.isfinite(distance)
    return mask
