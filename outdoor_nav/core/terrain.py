"""Deterministic terrain state machine, independent of ROS and model implementation."""
import math
from ..terrain_config import effective_priorities


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def flight_coordinates(point, flight):
    x, y = point[0]-flight['front_xy'][0], point[1]-flight['front_xy'][1]
    c, s = math.cos(flight['heading_yaw']), math.sin(flight['heading_yaw'])
    return c*x+s*y, -s*x+c*y


def observed_flight(obs):
    if not obs or not obs['stairs'] or obs['direction'] != 'up' or obs['heading_yaw'] is None:
        return None
    bounds = obs.get('flight_bounds')
    if bounds is None:
        return None
    flight = dict(front_xy=bounds['front_xy'], heading_yaw=obs['heading_yaw'])
    length, _ = flight_coordinates(bounds['end_xy'], flight)
    return dict(flight, length_m=length) if length > 0 else None


class TerrainMachine:
    """Four motion states; alignment/gait confirmation are upstairs action phases.

    Missing observations are unknown, never a negative stair observation. A fault
    while climbing latches STOP until navigation is disabled. No downstairs entry
    exists until a detector and descent policy have been commissioned.
    """
    def __init__(self, parameters):
        self.p = parameters
        self.reset()

    def reset(self):
        self.state, self.phase, self.reason = 'stop', 'idle', 'disabled'
        self.candidate_since = self.clear_since = self.phase_since = None
        self.target_yaw = self.candidate_heading = None
        self.flat_wait_since = None
        self.stair_seen_since = -math.inf
        self.stair_flight = self.next_flight = self.next_flight_since = None
        self.incline_hold = False
        self.failed = self.align_armed = False
        self.last_request = -math.inf

    def tick(self, now, *, enable, pitch=None, observation=None, yaw=None, position=None,
             actual_gait=None, gait_fresh=False, gait_ready=False, stopped=False):
        request, rotation = None, 0.
        if not enable:
            self.reset()
            return self.output()
        if self.failed:
            return self.output()
        if self.state == 'stop' and self.reason == 'disabled':
            self.state, self.phase, self.reason = 'flat', 'drive', 'enabled'
        if pitch is None or not math.isfinite(pitch):
            self.next_flight_since = None
            self.reason = 'imu_unavailable'
            return self.output(allowed=False)
        flat = abs(pitch) <= math.radians(self.p['pitch_flat_deg'])
        tilted = abs(pitch) >= math.radians(self.p['pitch_hold_deg'])
        if tilted:
            self.incline_hold = True
        elif flat:
            self.incline_hold = False
        obs = observation if observation and observation.get('valid') else None
        seen = bool(obs and obs.get('stairs'))
        evidence_time = obs.get('stamp', now) if obs else now
        if self.state in ('flat', 'stop'):
            if seen:
                self.clear_since = None
                if not self.p['can_climb_stairs']:
                    self.state, self.phase, self.reason = 'stop', 'blocked', 'stairs_unsupported'
                    return self.output()
                if not flat:
                    self.candidate_since = None
                    return self.output(allowed=False, reason='stairs_wait_level')
                # Flat gait may only close in on the steps from a distance;
                # inside the enter range it must stop until the machine has
                # confirmed, squared up and switched gait, otherwise the robot
                # would walk up the staircase in flat gait.
                close = self.close_stairs(obs)
                if obs.get('direction') != 'up':
                    self.candidate_since = None
                    return self.output(allowed=not close,
                                       reason='stair_stop_range' if close else 'stair_direction_pending')
                if obs.get('heading_yaw') is None:
                    self.candidate_since = None
                    return self.output(allowed=not close,
                                       reason='stair_stop_range' if close else 'stair_heading_pending')
                if self.candidate_heading is not None and abs(wrap(obs['heading_yaw']-self.candidate_heading)) > math.radians(self.p['alignment_tolerance_deg']):
                    self.candidate_since = None
                self.candidate_heading = obs['heading_yaw']
                self.candidate_since = evidence_time if self.candidate_since is None else self.candidate_since
                # Inside the enter range one good frame is enough: the robot is
                # already at the steps, waiting for a second frame only lets it
                # creep up in flat gait.
                if close or evidence_time - self.candidate_since >= self.p['confirm_s']:
                    self.begin_alignment(now, obs)
                else:
                    return self.output(allowed=not close,
                                       reason='stair_stop_range' if close else 'confirming_stairs')
            else:
                self.candidate_since = None
                if self.state == 'stop':
                    if flat and obs:
                        self.clear_since = evidence_time if self.clear_since is None else self.clear_since
                        if evidence_time-self.clear_since >= self.p['clear_s']:
                            self.state, self.phase, self.reason = 'flat', 'drive', 'stairs_cleared'
                    else:
                        self.clear_since = None
                    return self.output()
                self.reason = 'flat'
                # Enabled semantic sources must supply an explicit valid observation.
                if obs is None:
                    return self.output(reason='semantics_unavailable')
                if self.p['can_climb_stairs'] and not (gait_fresh and actual_gait == self.p['flat_gait_code']):
                    self.flat_wait_since = now if self.flat_wait_since is None else self.flat_wait_since
                    if now-self.flat_wait_since > self.p.get('gait_switch_timeout_s', 10.):
                        return self.fail('flat_gait_confirmation_timeout')
                    if stopped and gait_ready and now-self.last_request >= 1.:
                        self.last_request = now
                        return self.output(request=self.p['flat_gait_code'], reason='restore_flat_gait')
                    return self.output(reason='restore_flat_gait')
                self.flat_wait_since = None
                return self.output(allowed=True)
        if self.state == 'stairs_up':
            if obs is None:
                if self.phase == 'climb':
                    self.next_flight_since = None
                    if flat and self.next_flight is not None:
                        return self.output(reason='landing_observation_unavailable')
                # While climbing, the body lies parallel to the slope and the
                # camera cannot see the steps any more; a tilted IMU is proof
                # enough that the climb is still in progress. A frame or two
                # may also be missed on level ground before the climb starts.
                if self.phase == 'climb' and not flat:
                    return self.output(allowed=gait_fresh and actual_gait == self.p['upstairs_gait_code'], reason='stair_observation_gap')
                recent = now - getattr(self, 'stair_seen_since', -math.inf) <= self.p['observation_timeout_s']
                if recent:
                    return self.output(allowed=self.phase == 'climb' and gait_fresh and actual_gait == self.p['upstairs_gait_code'], reason='stair_observation_gap')
                return self.output(allowed=False, reason='stair_observation_unavailable')
            self.stair_seen_since = now
            if yaw is None or not math.isfinite(yaw):
                self.next_flight_since = None
                return self.output(allowed=False, reason='stair_observation_unavailable')
            if self.phase in ('align', 'switch_gait', 'climb'):
                following = self.track_flight(observed_flight(obs))
                landing = (self.phase == 'climb' and flat and gait_fresh
                           and actual_gait == self.p['upstairs_gait_code']
                           and self.passed_flight(following, position))
                if landing:
                    if not self.confirm_landing(following, evidence_time):
                        return self.output(reason='confirming_next_stair_flight')
                    self.phase, self.reason, self.phase_since = 'restore_flat', 'restore_flat_gait', now
                else:
                    self.next_flight = self.next_flight_since = None
            if flat and not seen:
                self.clear_since = evidence_time if self.clear_since is None else self.clear_since
                if evidence_time-self.clear_since >= self.p['clear_s']:
                    if self.phase != 'restore_flat':
                        self.phase, self.reason, self.phase_since = 'restore_flat', 'restore_flat_gait', now
            else:
                self.clear_since = None
            if self.phase == 'align':
                target = self.p['flat_gait_code']
                confirmed = gait_fresh and actual_gait == target
                if now-self.phase_since > self.p['alignment_timeout_s']:
                    return self.fail('alignment_timeout')
                if confirmed and (stopped or self.align_armed):
                    self.align_armed = True
                    error = wrap(self.target_yaw-yaw)
                    if abs(error) > math.radians(self.p['alignment_tolerance_deg']):
                        rotation = max(-self.p['alignment_speed_rps'], min(
                            self.p['alignment_speed_rps'], error*self.p['alignment_kp']))
                    elif stopped:
                        self.phase, self.phase_since = 'switch_gait', now
                elif stopped and gait_ready and now-self.last_request >= 1.:
                    request = target
            if self.phase in ('switch_gait', 'restore_flat'):
                target = self.p['upstairs_gait_code'] if self.phase == 'switch_gait' else self.p['flat_gait_code']
                landing_level = self.phase != 'restore_flat' or self.next_flight is None or flat
                if gait_fresh and actual_gait == target and stopped and landing_level:
                    if self.phase == 'restore_flat':
                        candidate = observed_flight(obs)
                        if (self.next_flight is not None and self.same_next_flight(candidate)
                                and self.passed_flight(candidate, position) and self.close_stairs(obs)):
                            self.begin_alignment(now, obs)
                            return self.output(reason='prepare_next_stair_flight')
                        self.state, self.phase, self.reason = 'flat', 'drive', 'stairs_complete'
                        self.stair_flight = self.next_flight = self.next_flight_since = None
                        self.candidate_since = self.candidate_heading = self.clear_since = None
                        return self.output(allowed=not seen)
                    self.phase, self.reason = 'climb', 'climbing'
                elif now-self.phase_since > self.p.get('gait_switch_timeout_s', 10.):
                    return self.fail('gait_confirmation_timeout')
                elif stopped and gait_ready and landing_level and now-self.last_request >= 1.:
                    request = target
            if request is not None:
                self.last_request = now
            allowed = self.phase == 'climb' and gait_fresh and actual_gait == self.p['upstairs_gait_code']
            return self.output(allowed=allowed, request=request, rotation=rotation)
        return self.output()

    def close_stairs(self, observation):
        front = observation.get('front_m')
        return front is not None and front <= self.p['stair_enter_range_m']

    def begin_alignment(self, now, obs):
        self.state, self.phase, self.reason = 'stairs_up', 'align', 'align_stair_front'
        self.target_yaw, self.phase_since, self.align_armed = obs['heading_yaw'], now, False
        self.stair_flight = observed_flight(obs)
        self.next_flight = self.next_flight_since = None
        self.clear_since = None

    def track_flight(self, observed):
        active = self.stair_flight
        if observed is None or active is None:
            return None
        along, lateral = flight_coordinates(observed['front_xy'], active)
        gap = self.p['stair_landing_gap_m']
        width = self.p['roi_half_width_m']
        separation = math.hypot(max(0., along-active['length_m']), max(0., abs(lateral)-width))
        if along >= 0. and separation >= gap:
            return observed
        heading_error = abs(wrap(observed['heading_yaw']-active['heading_yaw']))
        if abs(lateral) <= width and heading_error <= math.radians(self.p['alignment_tolerance_deg']):
            end = along + observed['length_m']*math.cos(heading_error)
            self.stair_flight = dict(active, length_m=max(active['length_m'], end))
        return None

    def passed_flight(self, following, position):
        return (following is not None and position is not None
                and flight_coordinates(position, self.stair_flight)[0] >= self.stair_flight['length_m']
                and flight_coordinates(position, following)[0] <= 0.)

    def same_next_flight(self, following):
        if following is None or self.next_flight is None:
            return False
        heading_error = abs(wrap(following['heading_yaw']-self.next_flight['heading_yaw']))
        return (math.dist(following['front_xy'], self.next_flight['front_xy']) <= self.p['stair_landing_gap_m']/2
                and heading_error <= math.radians(self.p['alignment_tolerance_deg']))

    def confirm_landing(self, following, stamp):
        if self.next_flight_since is None or not self.same_next_flight(following):
            self.next_flight, self.next_flight_since = following, stamp
        return stamp-self.next_flight_since >= self.p['confirm_s']

    def fail(self, reason):
        self.state, self.phase, self.reason, self.failed = 'stop', 'fault', reason, True
        return self.output()

    def output(self, allowed=False, reason=None, request=None, rotation=0.):
        substate = None
        if self.state == 'stairs_up':
            substate = 'climbing' if self.phase in ('climb', 'restore_flat') else 'preparing'
        return dict(state=self.state, phase=self.phase, reason=reason or self.reason,
                    motion_allowed=allowed, gait_request=request, angular_z=rotation,
                    substate=substate, target_yaw=self.target_yaw, failed=self.failed,
                    stair_flight=self.stair_flight if self.state == 'stairs_up' else None,
                    priorities=effective_priorities(self.p, self.state))
