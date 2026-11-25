"""Final command timeout independent of the navigation worker process."""

import json
import math
import time
import signal
from rclpy.signals import SignalHandlerOptions
from .configuration import load_runtime
from .core.scan_health import acquisition_monotonic, healthy_scan

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class Watchdog(Node):
    def __init__(self):
        super().__init__("lynx_command_watchdog")
        self.declare_parameter("robot_config", "")
        self.declare_parameter("diagnostic", False)
        self.diagnostic = self.get_parameter("diagnostic").value
        self.config = load_runtime(self.get_parameter("robot_config").value)["policy"]
        self.command, self.command_at, self.scan_at, self.geo_at = (
            Twist(),
            0.0,
            0.0,
            0.0,
        )
        self.ready = False
        self.camera_at = 0.0
        self.conflict = False
        self.pub = self.create_publisher(
            Twist, "/outdoor/diagnostic_cmd_vel" if self.diagnostic else "/cmd_vel", 10
        )
        self.status_pub = self.create_publisher(String, "/outdoor/command_status", 10)
        self.create_subscription(Twist, "/outdoor/safe_command", self.receive, 10)
        self.create_subscription(String, "/outdoor/localization", self.geo, 10)
        self.create_subscription(
            LaserScan, "/outdoor/scan", self.scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.tick)
        self.create_subscription(LaserScan, "/outdoor/camera_scan", self.camera_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.check_owners)
        if self.config.get("ranger_descent_enabled", False):
            from tf2_ros import Buffer, TransformListener
            from .core.ground_support import DESCENT_ACTIVE_TOPIC, SURVEY_ACTIVE_TOPIC
            from .ground_support_node import SupportFeed
            self.ranger_auto = self.config.get("ranger_descent_mode") == "auto"
            self.support_tf = Buffer()
            self.support_listener = TransformListener(self.support_tf, self)
            self.support = SupportFeed(self, self.config, self.support_tf, with_heights=self.ranger_auto)
            self.survey_support = (SupportFeed(self, self.config, self.support_tf, with_heights=True, survey=True)
                                   if self.ranger_auto else None)
            self.speed_sample = None
            self.support_epoch = None
            self.descent_at = -math.inf
            self.survey_at = -math.inf
            self.create_subscription(Odometry, "/outdoor/odom", self.ranger_odometry, 10)
            if self.ranger_auto:
                self.create_subscription(Bool, DESCENT_ACTIVE_TOPIC, self.ranger_descent_state, 1)
                self.create_subscription(Bool, SURVEY_ACTIVE_TOPIC, self.ranger_survey_state, 1)

    def check_owners(self):
        conflict = self.count_publishers(self.pub.topic_name) > 1
        if conflict != self.conflict:
            # Removing a competing publisher restores readiness, not an old
            # command: the active task must have survived and send fresh output.
            self.command, self.command_at = Twist(), 0.0
            if conflict:
                self.get_logger().error("Another command output publisher detected; stopping")
        self.conflict = conflict

    def receive(self, msg):
        self.command, self.command_at = msg, time.monotonic()

    def geo(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("Localization status must be an object")
            self.ready = (
                bool(data["ready"])
                and not data.get("ownership_conflict", False)
                and data.get("extrinsics", {}).get("ready", False)
            )
            self.geo_at = time.monotonic()
            if self.config.get("ranger_descent_enabled", False):
                epoch = ((data.get("fusion") or {}).get("recovery_id"),
                         (data.get("navigation_sample") or {}).get("epoch"))
                if self.support_epoch is not None and self.support_epoch != epoch:
                    self.support.invalidate()
                    if self.survey_support is not None:
                        self.survey_support.invalidate()
                    self.descent_at = -math.inf
                    self.survey_at = -math.inf
                self.support_epoch = epoch
        except (ValueError, KeyError):
            self.ready = False

    def scan(self, msg):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if healthy_scan(msg, now_s, self.config["sensor_timeout_s"]):
            self.scan_at = max(self.scan_at, acquisition_monotonic(msg, now_s, time.monotonic()))

    def ranger_odometry(self, msg):
        from .ground_support_node import seconds
        self.speed_sample = (msg.twist.twist.linear.x, seconds(msg.header.stamp), time.monotonic())
        self.ranger_yaw_rate = msg.twist.twist.angular.z

    def ranger_descent_state(self, msg):
        self.descent_at = time.monotonic() if msg.data else -math.inf

    def ranger_survey_state(self, msg):
        self.survey_at = time.monotonic() if msg.data else -math.inf

    def ranger_observation(self, now, *, feed=None):
        from tf2_ros import TransformException
        from .ground_support_node import latest_base, xy_yaw
        support = (feed if feed is not None else self.support).fresh()
        feedback = self.speed_sample
        timeout = self.config["ranger_depth_timeout_s"]
        if support is None or feedback is None:
            return None
        speed, stamp, received = feedback
        if now - received > timeout or not 0 <= self.get_clock().now().nanoseconds/1e9-stamp <= timeout:
            return None
        try:
            pose = latest_base(self.support_tf, self, timeout)
        except (TransformException, ValueError):
            return None
        xy, yaw = xy_yaw(pose)
        if not all(map(math.isfinite, (*xy, yaw, speed))):
            return None
        return support, xy, yaw, speed

    def ranger_permits(self, command, now):
        observation = self.ranger_observation(now)
        if observation is None:
            return False
        support, xy, yaw, speed = observation
        return support.permits(xy, yaw, command.linear.x, command.angular.z, speed, self.config)

    def ranger_limit(self, command, now):
        from .core.ground_support import SEGMENT_SPEED, braking_paths, braking_speed
        timeout = self.config["command_timeout_s"]
        survey = now - getattr(self, "survey_at", -math.inf) < timeout
        descent = now - self.descent_at < timeout
        if descent and not survey:
            return Twist(), "ranger_mode_unconfirmed"
        if survey and not descent and command.linear.x:
            return Twist(), "ranger_survey_waiting_for_segment"
        feed = self.survey_support if survey else self.support
        observation = self.ranger_observation(now, feed=feed)
        if observation is None:
            return Twist(), "ranger_ground_stale"
        support, xy, yaw, measured = observation
        heights = feed.elevation(support)
        if heights is None:
            return Twist(), "ranger_height_unpaired"
        speed, turn = command.linear.x, command.angular.z
        if speed == 0.:
            if abs(measured) >= .01 or not support.clear([xy]):
                return Twist(), "ranger_braking_or_ground_unconfirmed"
            return command, None
        if measured < -.01 or not math.isfinite(self.ranger_yaw_rate):
            return Twist(), "ranger_velocity_unconfirmed"
        latency = self.config["ranger_depth_timeout_s"] + .05 + self.config["ranger_actuation_latency_s"]
        decel = self.config["ranger_brake_mps2"]
        radius = self.config["footprint_radius_m"]
        stopping = abs(measured) * latency + measured**2 / (2 * decel)
        horizon = radius + max(.9, stopping + .55)
        available, reason = horizon, "ranger_ground_unconfirmed"
        for points in braking_paths(xy, yaw, speed, turn, abs(measured), self.ranger_yaw_rate, horizon):
            profile = support.profile(xy, points, heights, detect_slope=not descent)
            distance = profile.supported_m
            why = profile.reason
            if not descent and profile.crest_m is not None:
                distance, why = min(distance, max(0., profile.crest_m - radius)), "ranger_crest"
            if distance < available:
                available, reason = distance, why
        if not descent and available <= .8 and reason == "ground_hazard":
            self.ranger_terrain_ahead = reason
        if descent and measured > SEGMENT_SPEED:
            return Twist(), "ranger_descent_overspeed"
        cap = braking_speed(available, abs(measured), min(speed, SEGMENT_SPEED) if descent else speed,
                            latency, decel)
        if cap <= 0.:
            return Twist(), reason
        command.linear.x = cap
        command.angular.z *= cap / speed
        return command, None

    def tick(self):
        now = time.monotonic()
        self.ranger_terrain_ahead = None
        output = Twist()
        checks = {
            "output_conflict": not self.conflict,
            "diagnostic_mode": not self.diagnostic,
            "motion_disabled": self.config["motion_enabled"],
            "localization_unready": self.ready,
            "localization_stale": now - self.geo_at < self.config.get("localization_timeout_s", 3.0),
            "scan_stale": not self.config.get("lidar_enabled", True) or now - self.scan_at < self.config["sensor_timeout_s"],
            # The camera obstacle scan only exists when the depth-geometry chain
            # runs; segmentation-only camera use does not publish one.
            "camera_stale": not (self.config.get("camera_enabled", False)
                                 and self.config.get("camera_geometry_enabled", True))
                             or now-getattr(self, "camera_at", 0) < self.config["sensor_timeout_s"],
            "command_stale": now - self.command_at < self.config["command_timeout_s"],
        }
        blockers = [reason for reason, passed in checks.items() if not passed]
        if not blockers:
            x, y, z = (
                self.command.linear.x,
                self.command.linear.y,
                self.command.angular.z,
            )
            if all(math.isfinite(v) for v in (x, y, z)):
                # No reverse or lateral motion until rear coverage is validated.
                output.linear.x = max(0.0, min(self.config["max_speed_mps"], x))
                output.angular.z = max(
                    -self.config["max_angular_speed_rps"],
                    min(self.config["max_angular_speed_rps"], z),
                )
            else:
                blockers.append("nonfinite_command")
        if self.config.get("ranger_descent_enabled", False):
            if getattr(self, "ranger_auto", False):
                if not blockers and (output.linear.x or output.angular.z):
                    output, reason = self.ranger_limit(output, now)
                    if reason:
                        blockers.append(reason)
            else:
                if not blockers and (output.linear.x or output.angular.z) and not self.ranger_permits(output, now):
                    blockers.append("ranger_ground_unconfirmed")
                    output = Twist()
                from .core.ground_support import SEGMENT_SPEED
                output.linear.x = min(output.linear.x, SEGMENT_SPEED)
        self.pub.publish(output)
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "conflict": self.conflict,
                        "diagnostic": self.diagnostic,
                        "output_topic": self.pub.topic_name,
                        "blocked_reasons": blockers,
                        "ranger_terrain_ahead": self.ranger_terrain_ahead,
                    }
                )
            )
        )

    def camera_scan(self, msg):
        if not msg.header.frame_id:
            self.camera_at = -math.inf
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        if healthy_scan(msg, now_s,
                        self.config["sensor_timeout_s"], allow_filtered_empty=True):
            self.camera_at = max(self.camera_at, acquisition_monotonic(msg, now_s, time.monotonic()))

    def destroy_node(self):
        if rclpy.ok():
            self.pub.publish(Twist())
        super().destroy_node()


def main():
    # Publish the final zero before shutting down the ROS context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Watchdog()
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
