"""Stable decision-cycle configuration; servo and watchdog retain their rates."""
import math

DEFAULTS = dict(enabled=False, period_s=2., hint_spacing_m=6., corner_error_m=.3,
                arrival_side_m=3., lookahead_m=3., grid_resolution_m=.1,
                grid_half_width_m=8., sample_limit=512)


def validate_cycle(raw):
    if not isinstance(raw, dict) or set(raw)-DEFAULTS.keys():
        raise ValueError('Unknown decision_cycle fields')
    c = {**DEFAULTS, **raw}
    if type(c['enabled']) is not bool:
        raise ValueError('decision_cycle.enabled must be boolean')
    for k in DEFAULTS.keys()-{'enabled'}:
        if type(c[k]) not in (int,float) or not math.isfinite(c[k]) or c[k] <= 0:
            raise ValueError('Invalid decision_cycle.'+k)
    if not (.2 <= c['period_s'] <= 10 and c['hint_spacing_m'] >= 6
            and .03 <= c['grid_resolution_m'] <= .25
            and c['lookahead_m'] < c['grid_half_width_m'] <= 20
            and type(c['sample_limit']) is int and 2 <= c['sample_limit'] <= 4096):
        raise ValueError('Invalid cycle period, spacing, grid size or sample bound')
    return c
