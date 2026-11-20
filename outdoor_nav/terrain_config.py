"""Optional terrain supervisor; hardware codes and model classes are deployment data."""
from copy import deepcopy
import math

SOURCES = ('lidar_obstacles', 'lidar_semantics', 'visual_obstacles', 'visual_semantics')
DEFAULTS = dict(
    enabled=False, slope_filter_enabled=False, lidar_slope_enabled=True,
    camera_slope_enabled=True, slope_angle_deg=15., ground_z_m=None,
    height_margin_m=.10, max_obstacle_height_m=2.5,
    can_climb_stairs=False, stairs_classes=[], lidar_semantics_enabled=False,
    pitch_flat_deg=4., pitch_hold_deg=8., pitch_sign=1., pitch_offset_deg=0.,
    confirm_s=.5, clear_s=.8, observation_timeout_s=1.,
    roi_half_width_m=.8, min_stair_points=20, min_stair_rise_m=.08,
    front_band_m=.2, min_front_width_m=.35, max_front_residual_m=.08,
    alignment_tolerance_deg=6., alignment_speed_rps=.15, alignment_kp=.8,
    alignment_timeout_s=20., stairs_speed_mps=.15, semantic_match_radius_m=.12,
    flat_gait_code=None, upstairs_gait_code=None,
    priorities=dict(flat=dict(lidar_obstacles=100, visual_obstacles=90,
                             lidar_semantics=50, visual_semantics=40),
                    stairs_up=dict(visual_semantics=100, lidar_semantics=90,
                                   lidar_obstacles=60, visual_obstacles=50)))


def validate_terrain(raw, policy, segmentation, gait_enabled):
    if not isinstance(raw, dict) or set(raw) - DEFAULTS.keys():
        raise ValueError('Unknown terrain configuration fields')
    p = {**deepcopy(DEFAULTS), **deepcopy(raw)}
    ground = p["ground_z_m"]
    if type(ground) not in (float, int) or not math.isfinite(ground) or ground >= 0:
        raise ValueError("terrain.ground_z_m must be supplied by the machine-local ground_height_file")
    for key, default in DEFAULTS.items():
        value = p[key]
        if type(default) is bool and type(value) is not bool:
            raise ValueError('terrain.' + key + ' must be boolean')
        if type(default) in (int, float):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('terrain.' + key + ' must be finite')
            if key not in ('ground_z_m', 'pitch_sign', 'pitch_offset_deg') and value <= 0:
                raise ValueError('terrain.' + key + ' must be positive')
    if not (0 < p['slope_angle_deg'] < 45 and p['ground_z_m'] < 0
            and p['height_margin_m'] < p['max_obstacle_height_m']
            and 0 < p['pitch_flat_deg'] < p['pitch_hold_deg'] < 90
            and p['pitch_sign'] in (-1, 1)
            and p['alignment_speed_rps'] <= policy['max_angular_speed_rps']
            and p['stairs_speed_mps'] <= policy['max_speed_mps']
            and type(p['min_stair_points']) is int and p['min_stair_points'] >= 3):
        raise ValueError('Invalid terrain geometry, pitch hysteresis or speed limits')
    if (not isinstance(p['stairs_classes'], list)
            or any(not isinstance(x, str) for x in p['stairs_classes'])
            or set(p['stairs_classes']) - set(segmentation['classes'].values())):
        raise ValueError('terrain.stairs_classes must name configured model classes')
    for key in ('flat_gait_code', 'upstairs_gait_code'):
        if p[key] is not None and (type(p[key]) is not int or not 0 <= p[key] <= 0xffffffff):
            raise ValueError('terrain.' + key + ' must be a UInt32 or null')
    if p['lidar_semantics_enabled']:
        raise ValueError('TODO: lidar semantics is not implemented; keep terrain.lidar_semantics_enabled=false. Other sources work independently.')
    if (p['slope_filter_enabled'] and p['camera_slope_enabled'] and policy['camera_enabled']
            and abs(p['ground_z_m']-policy['camera_expected_ground_z_m']) > policy['camera_ground_tolerance_m']):
        raise ValueError('Shared ground_z_m must agree with calibrated camera ground height')
    if p['enabled'] and p['can_climb_stairs']:
        if (not gait_enabled or p['flat_gait_code'] is None or p['upstairs_gait_code'] is None
                or p['flat_gait_code'] == p['upstairs_gait_code']):
            raise ValueError('Stair capability requires guarded gait output and distinct explicit gait codes')
        if not ((policy['camera_enabled'] and segmentation['enabled'] and p['stairs_classes'])
                or p['lidar_semantics_enabled']):
            raise ValueError('Stair capability requires a configured semantic source')
    raw_priorities = raw.get('priorities', {})
    if not isinstance(raw_priorities, dict) or set(raw_priorities) - {'flat', 'stairs_up'}:
        raise ValueError('terrain.priorities accepts flat and stairs_up tables')
    priorities = deepcopy(DEFAULTS['priorities'])
    for state, values in raw_priorities.items():
        if (not isinstance(values, dict) or set(values) - set(SOURCES)
                or any(type(v) is not int or not 0 <= v <= 1000 for v in values.values())):
            raise ValueError('Terrain priorities require known sources and integer ranks in [0, 1000]')
        priorities[state].update(values)
    p['priorities'] = priorities
    p['active_sources'] = active_sources(policy, segmentation)
    return p


def active_sources(policy, segmentation):
    """Effective sources only; lidar semantics remains TODO."""
    result = []
    if policy['lidar_enabled']:
        result.append('lidar_obstacles')
    if policy['camera_enabled'] and policy['camera_geometry_enabled']:
        result.append('visual_obstacles')
    if policy['camera_enabled'] and segmentation['enabled']:
        result.append('visual_semantics')
    return result


def effective_priorities(parameters, state):
    table = parameters['priorities']['stairs_up' if state == 'stairs_up' else 'flat']
    enabled = parameters.get('active_sources', [s for s in SOURCES if s != 'lidar_semantics'])
    return {source: table[source] for source in enabled}
