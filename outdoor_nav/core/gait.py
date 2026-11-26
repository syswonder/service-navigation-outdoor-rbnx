"""Ordered route station and feedback-confirmed gait requests; no motion-state changes."""

from copy import deepcopy
import math

from .guidance import (arrived_at_checkpoint, distance, projection, entered_next_segment,
                        gait_boundary_points, insert_checkpoints)

FLAT = 0x3002
STAIRS = 0x3003
RAW_GAITS = {"flat": (0x1001, FLAT), "stairs": (0x1003, STAIRS)}


def chassis_prepared(body, received_age):
    """Mode and feedback permit a gait request; actual gait gates velocity later."""
    ages = ((received_age, 1.), (body.get("motion_feedback_age_s"), 1.),
            (body.get("usage_mode_age_s"), 3.))
    return (body.get("state") == 17 and body.get("usage_mode") == 1
            and body.get("gait_output_enabled") is True
            and all(type(age) in (int, float) and math.isfinite(age) and 0 <= age < limit
                    for age, limit in ages))


def route_length(reference):
    return sum(distance(a, b) for a, b in zip(reference, reference[1:]))


def validate_intervals(reference, intervals):
    if not isinstance(intervals, list) or len(intervals) > 8192:
        raise ValueError("intervals must be a bounded list")
    total, previous, result = route_length(reference), 0.0, []
    for raw in intervals:
        if not isinstance(raw, dict):
            raise ValueError("A gait interval must be an object")
        a, b = raw.get("start_m"), raw.get("end_m")
        if not all(type(x) in (int, float) and math.isfinite(x) for x in (a, b)):
            raise ValueError("Gait interval stations must be finite")
        if not 0 <= a < b <= total + 1e-6 or a < previous - 1e-6:
            raise ValueError("Gait intervals must follow the submitted route arc length")
        gait, code = raw.get("gait"), raw.get("gait_code")
        if gait not in (*RAW_GAITS, "unknown"):
            raise ValueError("Unknown gait annotation")
        if code is not None and (type(code) is not int or not 0 <= code <= 0xffffffff):
            raise ValueError("Invalid actual gait code")
        if gait in RAW_GAITS and code not in RAW_GAITS[gait]:
            raise ValueError("Gait label and recorded code disagree")
        if gait == "unknown" and any(code in codes for codes in RAW_GAITS.values()):
            raise ValueError("Known gait code cannot be labelled unknown")
        previous = b
        result.append({**deepcopy(raw), "start_m": float(a), "end_m": float(b)})
    return result


class RouteStation:
    def __init__(self, reference, checkpoints=(), checkpoint_tolerance=.25):
        reference = insert_checkpoints(reference, checkpoints)
        self.checkpoint_tolerance = checkpoint_tolerance
        self.points = [reference[0]]
        for point in reference[1:]:
            if distance(self.points[-1], point) > 1e-6:
                self.points.append(point)
        self.offsets = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.offsets.append(self.offsets[-1] + distance(a, b))
        self.index, self.station = 0, 0.0
        self.checkpoints = {i for i, point in enumerate(self.points)
                            if any(distance(point, p) < 1e-6 for p in checkpoints)}
        self.passed = set()

    def update(self, point):
        if len(self.points) < 2:
            return 0.0
        while self.index + 2 < len(self.points):
            a, b, c = self.points[self.index:self.index + 3]
            boundary = self.index + 1
            if boundary in self.checkpoints and boundary not in self.passed:
                if not arrived_at_checkpoint(
                    point, a, b, self.checkpoint_tolerance
                ):
                    break
                self.passed.add(boundary)
                self.index += 1
                continue
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            px, py = point["x"] - b["x"], point["y"] - b["y"]
            crossed = px * dx + py * dy >= 0
            lateral = abs(px * dy - py * dx) / math.hypot(dx, dy)
            turning = entered_next_segment(point, a, b, c)
            if not ((crossed and lateral <= 1.5) or turning):
                break
            self.index += 1
        current = projection(point, self.points[self.index:self.index + 2])
        station = self.offsets[self.index] + current["offset_m"]
        boundary = self.index + 1
        if boundary in self.checkpoints and boundary not in self.passed:
            station = min(station, self.offsets[boundary]-1e-6)
        self.station = max(self.station, station)
        return self.station


class GaitSchedule:
    def __init__(self, reference, intervals, checkpoint_tolerance=.25):
        self.intervals = validate_intervals(reference, intervals)
        self.route = RouteStation(reference, gait_boundary_points(reference, self.intervals), checkpoint_tolerance)
        self.stairs = []
        total = route_length(reference)
        for interval in self.intervals:
            if interval["gait"] != "stairs":
                continue
            a, b = max(0., interval["start_m"] - 1.), min(total, interval["end_m"] + 1.)
            if self.stairs and a <= self.stairs[-1][1]:
                self.stairs[-1][1] = max(b, self.stairs[-1][1])
            else:
                self.stairs.append([a, b])
        self.target = None
        self.last_sent, self.attempts = -math.inf, 0
        self.switch_started = None
        self.prepare_started = None
        self.failed = False
        # The robot ignores pure-rotation commands in the stairs gait, so a
        # rotation on a stairs section temporarily overrides the target to the
        # flat gait: switch to flat, rotate, then switch back to stairs.
        self.rotation_hold = False
        self.rotation_since = None

    def desired(self, station):
        if any(a <= station < b or (station == b == self.route.offsets[-1])
               for a, b in self.stairs):
            return STAIRS
        for item in self.intervals:
            if item["start_m"] <= station <= item["end_m"]:
                return FLAT if item["gait"] == "flat" else None
        return None

    def tick(self, position, now, actual, feedback_age_s, eligible, stopped=False, timeout_s=5., prepare_timeout_s=15., rotating=False):
        station = self.route.update(position)
        desired = self.desired(station)
        # Only a robot that has actually stopped may trade the stairs gait for
        # the flat one to turn in place. A moving climb emits small rotation
        # corrections all the time; treating those as in-place turns flipped the
        # gait every few seconds and the robot never got up a step.
        if rotating and desired == STAIRS and stopped:
            if not self.rotation_hold:
                self.rotation_hold = True
            self.rotation_since = now
        elif self.rotation_hold and (not rotating and self.rotation_since is not None
                                     and now - self.rotation_since >= 1.0):
            self.rotation_hold = False
            self.rotation_since = None
        target = FLAT if self.rotation_hold and desired == STAIRS else desired
        if target != self.target:
            self.target, self.last_sent, self.attempts = target, -math.inf, 0
            self.switch_started = self.prepare_started = None
        fresh = math.isfinite(feedback_age_s) and 0 <= feedback_age_s < 1.
        confirmed = target is not None and fresh and actual == target
        if confirmed:
            self.switch_started = self.prepare_started = None
        elif self.prepare_started is None:
            self.prepare_started = now
        preparing_expired = (self.switch_started is None and self.prepare_started is not None
                             and now - self.prepare_started >= prepare_timeout_s)
        confirmation_expired = (self.switch_started is not None
                                and now - self.switch_started >= timeout_s)
        if target is None or preparing_expired or confirmation_expired:
            self.failed = True
        command = None
        if not self.failed and not confirmed and eligible and stopped and now - self.last_sent >= 1.:
            command, self.last_sent = target, now
            if self.switch_started is None:
                self.switch_started = now
            self.attempts += 1
        reason = ("unknown_route_gait" if target is None else
                  "gait_prepare_timeout" if self.failed and preparing_expired else
                  "gait_switch_timeout" if self.failed else
                  None if confirmed else
                  "stopping_before_gait_switch" if not stopped else
                  "waiting_for_eligible_state" if not eligible else
                  "gait_feedback_stale" if not fresh else "gait_not_confirmed_retrying")
        return command, {
            "automatic": True, "target_code": target, "actual_code": actual,
            "confirmed": confirmed, "feedback_fresh": fresh, "attempts": self.attempts,
            "route_progress_m": station, "reason": reason,
            "failed": self.failed, "motion_allowed": confirmed and not self.failed,
            "local_avoidance_enabled": target != STAIRS,
            "rotation_hold": self.rotation_hold,
        }
