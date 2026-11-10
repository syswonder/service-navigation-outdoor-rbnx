"""Shared interpretation of legacy and inertial localization states."""


def continuation_allowed(status, owned, allow_float=True):
    grade = status.get("grade")
    if not status.get("ready"):
        return False
    if grade == "inertial":
        return bool(
            owned
            and status.get("fusion", {}).get("estimate_valid")
            and status.get("fusion", {}).get("navigation_ready")
        )
    if grade not in ("fixed", "float"):
        return False
    return (
        not owned
        or owned.get("localization_grade") != "fixed"
        or grade == "fixed"
        or allow_float
    )


def arrival_confirmed(status):
    return status.get("ready") and status.get("grade") in ("fixed", "float")
