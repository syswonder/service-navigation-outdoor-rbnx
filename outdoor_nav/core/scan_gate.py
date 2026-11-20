"""Decide when stair treads must not be treated as lidar obstacles."""


def stairs_obstacle_off(gait_status, enabled=True):
    """True once a stair gait has been requested and is being retried or confirmed.

    The request starts one metre before the stair section, so treads are
    ignored before the first step rather than after it.
    """
    if not enabled or not isinstance(gait_status, dict):
        return False
    return gait_status.get("target_code") == 0x3003 and (
        gait_status.get("confirmed") or (gait_status.get("attempts") or 0) > 0
    )
