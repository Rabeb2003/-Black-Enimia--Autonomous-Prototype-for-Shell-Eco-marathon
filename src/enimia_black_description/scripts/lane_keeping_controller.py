#!/usr/bin/env python3
"""
Lane Keeping Controller: Seamlessly blends Nav2 Goal Navigation with Real-Time Lane Centering.

Role:
- Subscribes to Nav2's smoothed velocity commands (`/cmd_vel_safe`).
- Subscribes to real-time lane cross-track error (`/lane_detection/center_error`)
  and preview heading error (`/lane_detection/heading_error`).
- Constrains Nav2's trajectory so the vehicle stays precisely centered in the circuit
  and never exits the track borders.
- Automatically regulates speed on sharp bends and large cross-track deviations to prevent skidding.
- Respects vehicle mechanical Ackermann curvature limits.
- Publishes to `/cmd_vel_lane`.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class LaneKeepingController(Node):
    def __init__(self):
        super().__init__('lane_keeping_controller')

        # Parameters
        self.declare_parameter('input_cmd_topic', '/cmd_vel_safe')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_lane')
        self.declare_parameter('lane_error_topic', '/lane_detection/center_error')
        self.declare_parameter('heading_error_topic', '/lane_detection/heading_error')
        self.declare_parameter('lane_valid_topic', '/lane_detection/valid')
        self.declare_parameter('stop_topic', '/stop_line/detected')
        self.declare_parameter('stop_sign_topic', '/stop_sign/detected')
        self.declare_parameter('curve_pose_topic', '/amcl_pose')
        self.declare_parameter('curve_fallback_pose_topic', '/odometry/filtered')
        self.declare_parameter('curve_reference_path_topic', '/lane_keeping/kr_in_reference_path')
        self.declare_parameter('curve_paint_path_topic', '/lane_keeping/kr_in_paint_reference_path')

        # 5.6 m/s is approximately 20 km/h.  The final safety gate still
        # has priority for the yellow stop line and STOP sign.
        self.declare_parameter('target_speed', 5.6)
        self.declare_parameter('max_speed', 6.0)
        self.declare_parameter('min_curve_speed', 3.8)
        self.declare_parameter('max_acceleration', 7.0)
        self.declare_parameter('max_deceleration', 10.0)  # Quick deceleration into curves
        self.declare_parameter('max_curvature', 0.30)      # 0.52 rad steering permits 0.307 1/m mechanically

        # Control gains tuned for sharp track bends and instant reaction
        self.declare_parameter('kp', 1.15)                 # Strong lateral centering
        self.declare_parameter('kd', 0.12)                 # Derivative damping
        self.declare_parameter('kh', 0.85)                 # Strong turn anticipation
        self.declare_parameter('right_curve_gain', 1.70)   # Extra anticipation for right bends
        self.declare_parameter('right_curve_bias', 0.055)  # Right-turn feed-forward
        self.declare_parameter('right_curve_threshold', 0.04)

        # The white paint is weak around the first right-hand turn.  The
        # kR_in markers form its *inner* painted edge.  Starting at k19 gives
        # the controller enough distance to converge before the critical
        # k14 -> k0 portion requested for this bend.
        self.declare_parameter('kr_in_curve_points', [
            78.9390, -7.5833, 79.3843, -7.9758,
            79.7906, -8.4085, 80.1543, -8.8775,
            80.4724, -9.3786,
            80.7418, -9.9075, 80.9603, -10.4593,
            81.1259, -11.0293, 81.2371, -11.6123,
            81.2930, -12.2032, 81.2930, -12.7968,
            81.2371, -13.3877, 81.1259, -13.9707,
            80.9603, -14.5407, 80.7418, -15.0925,
            80.4724, -15.6214, 80.1543, -16.1225,
            79.7906, -16.5915, 79.3843, -17.0242,
            78.9390, -17.4167,
        ])
        # kR_in is the inner kerb, not the centre line.  The road is wide at
        # this location; 2.90 m puts the vehicle on its actual middle line,
        # between the white boundaries rather than near the inner grass.
        self.declare_parameter('kr_in_safety_offset', 2.90)
        self.declare_parameter('kr_in_curve_center_x', 75.0)
        self.declare_parameter('kr_in_curve_center_y', -12.5)
        # road_R_jnt62 is the real physical start of the right bend.  Do not
        # estimate entry from the distant kR_in paint; that was engaging too
        # early and cutting over the white boundary.
        self.declare_parameter('kr_in_trigger_x', 85.3630)
        self.declare_parameter('kr_in_trigger_y', -5.5101)
        self.declare_parameter('kr_in_trigger_radius', 3.8)
        self.declare_parameter('kr_in_exit_radius', 1.1)
        self.declare_parameter('kr_in_lookahead', 2.4)
        self.declare_parameter('kr_in_speed', 4.2)
        # Geometric lane-tracking gains for the poorly painted first bend.
        # They combine the reference tangent, lateral offset and curvature
        # instead of relying on camera pixels alone.
        self.declare_parameter('kr_in_heading_gain', 1.70)
        self.declare_parameter('kr_in_cross_track_gain', 1.25)
        self.declare_parameter('kr_in_pure_pursuit_weight', 0.45)

        self.declare_parameter('lane_timeout', 0.8)        # Seconds before lane signal considered lost
        self.declare_parameter('command_timeout', 0.5)     # Nav2 command freshness timeout
        self.declare_parameter('publish_rate', 30.0)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.lane_error_topic = self.get_parameter('lane_error_topic').value
        self.heading_error_topic = self.get_parameter('heading_error_topic').value
        self.lane_valid_topic = self.get_parameter('lane_valid_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.stop_sign_topic = self.get_parameter('stop_sign_topic').value
        self.curve_pose_topic = self.get_parameter('curve_pose_topic').value
        self.curve_fallback_pose_topic = self.get_parameter(
            'curve_fallback_pose_topic').value
        self.curve_reference_path_topic = self.get_parameter(
            'curve_reference_path_topic').value
        self.curve_paint_path_topic = self.get_parameter(
            'curve_paint_path_topic').value

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_curve_speed = float(self.get_parameter('min_curve_speed').value)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.max_deceleration = float(self.get_parameter('max_deceleration').value)
        self.max_curvature = float(self.get_parameter('max_curvature').value)

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.kh = float(self.get_parameter('kh').value)
        self.right_curve_gain = float(self.get_parameter('right_curve_gain').value)
        self.right_curve_bias = float(self.get_parameter('right_curve_bias').value)
        self.right_curve_threshold = float(
            self.get_parameter('right_curve_threshold').value)
        curve_values = [float(value) for value in
                        self.get_parameter('kr_in_curve_points').value]
        if len(curve_values) < 4 or len(curve_values) % 2:
            raise ValueError('kr_in_curve_points must contain x/y pairs')
        self.kr_in_paint_points = list(zip(curve_values[0::2], curve_values[1::2]))
        self.kr_in_safety_offset = float(
            self.get_parameter('kr_in_safety_offset').value)
        curve_center_x = float(self.get_parameter('kr_in_curve_center_x').value)
        curve_center_y = float(self.get_parameter('kr_in_curve_center_y').value)
        if self.kr_in_safety_offset < 0.0:
            raise ValueError('kr_in_safety_offset must be >= 0')

        # The coordinates in the world are expressed in the same map frame.
        # Offset each kR_in sample radially outwards to obtain the vehicle
        # centre trajectory.  Following the painted edge itself would put the
        # chassis on the kerb; this parallel curve follows the middle lane.
        self.kr_in_curve_points = []
        for point_x, point_y in self.kr_in_paint_points:
            radial_x = point_x - curve_center_x
            radial_y = point_y - curve_center_y
            radial_length = math.hypot(radial_x, radial_y)
            if radial_length < 1.0e-6:
                raise ValueError('kr_in curve point cannot equal its centre')
            self.kr_in_curve_points.append((
                point_x + self.kr_in_safety_offset * radial_x / radial_length,
                point_y + self.kr_in_safety_offset * radial_y / radial_length,
            ))
        self.kr_in_trigger_x = float(
            self.get_parameter('kr_in_trigger_x').value)
        self.kr_in_trigger_y = float(
            self.get_parameter('kr_in_trigger_y').value)
        self.kr_in_trigger_radius = float(
            self.get_parameter('kr_in_trigger_radius').value)
        self.kr_in_exit_radius = float(
            self.get_parameter('kr_in_exit_radius').value)
        self.kr_in_lookahead = float(
            self.get_parameter('kr_in_lookahead').value)
        self.kr_in_speed = float(self.get_parameter('kr_in_speed').value)
        self.kr_in_heading_gain = float(
            self.get_parameter('kr_in_heading_gain').value)
        self.kr_in_cross_track_gain = float(
            self.get_parameter('kr_in_cross_track_gain').value)
        self.kr_in_pure_pursuit_weight = float(
            self.get_parameter('kr_in_pure_pursuit_weight').value)
        self.kr_in_pure_pursuit_weight = max(
            0.0, min(1.0, self.kr_in_pure_pursuit_weight))

        self.lane_timeout = float(self.get_parameter('lane_timeout').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # State tracking
        self.latest_nav_cmd = Twist()
        self.last_nav_command_at = 0.0

        self.lane_error = 0.0
        self.previous_lane_error = 0.0
        self.heading_error = 0.0
        self.last_lane_at = 0.0
        self.lane_valid = False

        self.stop_detected = False
        self.stop_sign_detected = False
        self.last_output_speed = 0.0
        self.last_output_at = time.monotonic()
        self.map_pose = None
        self.map_pose_at = 0.0
        self.fallback_pose = None
        self.fallback_pose_at = 0.0
        self.using_fallback_curve_pose = False
        self.kr_in_active = False
        self.kr_in_index = 0

        # ROS 2 Subscriptions & Publishers
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.curve_path_pub = self.create_publisher(
            Path, self.curve_reference_path_topic, 1)
        self.curve_paint_path_pub = self.create_publisher(
            Path, self.curve_paint_path_topic, 1)
        self.curve_cross_track_pub = self.create_publisher(
            Float32, '/lane_keeping/kr_in_cross_track_error', 10)
        self.curve_heading_pub = self.create_publisher(
            Float32, '/lane_keeping/kr_in_heading_error', 10)
        self.curve_curvature_pub = self.create_publisher(
            Float32, '/lane_keeping/kr_in_curvature', 10)
        self.create_subscription(Twist, self.input_cmd_topic, self.nav_cmd_callback, 10)
        self.create_subscription(Float32, self.lane_error_topic, self.lane_error_callback, 10)
        self.create_subscription(Float32, self.heading_error_topic, self.heading_error_callback, 10)
        self.create_subscription(Bool, self.lane_valid_topic, self.lane_valid_callback, 10)
        self.create_subscription(Bool, self.stop_topic, self.stop_callback, 10)
        self.create_subscription(Bool, self.stop_sign_topic, self.stop_sign_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.curve_pose_topic,
            self.curve_pose_callback, 10)
        # The P3D/AMCL pose is the normal reference.  Wheel/IMU EKF odometry
        # is a short-range fallback so the first kR_in turn is still executed
        # if Gazebo's optional P3D plugin has not started publishing yet.
        self.create_subscription(
            Odometry, self.curve_fallback_pose_topic,
            self.curve_fallback_pose_callback, 20)

        self.create_timer(1.0 / publish_rate, self.publish_command)
        # RViz may start after this node. Re-publish the static reference so
        # the blue kR_in guide is always visible to the operator.
        self.create_timer(1.0, self.publish_kr_in_reference_path)

        self.get_logger().info(
            f'Lane Keeping Controller active: {self.input_cmd_topic} + {self.lane_error_topic} -> {self.output_cmd_topic}'
        )

    def publish_kr_in_reference_path(self):
        """Publish the safe target and its kR_in painted-edge reference."""
        self.curve_path_pub.publish(self.make_path(self.kr_in_curve_points))
        self.curve_paint_path_pub.publish(self.make_path(self.kr_in_paint_points))

    def make_path(self, points):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        last_index = len(points) - 1

        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.04

            if index < last_index:
                next_x, next_y = points[index + 1]
                heading = math.atan2(next_y - y, next_x - x)
            else:
                prev_x, prev_y = points[index - 1]
                heading = math.atan2(y - prev_y, x - prev_x)
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path.poses.append(pose)

        return path

    def nav_cmd_callback(self, msg: Twist):
        self.latest_nav_cmd = msg
        self.last_nav_command_at = time.monotonic()

    def lane_error_callback(self, msg: Float32):
        self.previous_lane_error = self.lane_error
        self.lane_error = float(max(-1.0, min(1.0, msg.data)))

    def heading_error_callback(self, msg: Float32):
        self.heading_error = float(max(-1.0, min(1.0, msg.data)))

    def lane_valid_callback(self, msg: Bool):
        self.lane_valid = bool(msg.data)
        if self.lane_valid:
            self.last_lane_at = time.monotonic()

    def stop_callback(self, msg: Bool):
        self.stop_detected = bool(msg.data)

    def stop_sign_callback(self, msg: Bool):
        self.stop_sign_detected = bool(msg.data)

    def curve_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.map_pose = self.pose_to_xy_yaw(msg.pose.pose)
        self.map_pose_at = time.monotonic()

    @staticmethod
    def pose_to_xy_yaw(pose):
        quaternion = pose.orientation
        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return (
            float(pose.position.x), float(pose.position.y),
            math.atan2(sin_yaw, cos_yaw),
        )

    def curve_fallback_pose_callback(self, msg: Odometry):
        self.fallback_pose = self.pose_to_xy_yaw(msg.pose.pose)
        self.fallback_pose_at = time.monotonic()

    def curve_pose_for_assist(self, now):
        """Return the freshest usable pose and whether it is EKF fallback."""
        if self.map_pose is not None and now - self.map_pose_at <= 0.75:
            self.using_fallback_curve_pose = False
            return self.map_pose
        if (self.fallback_pose is not None and
                now - self.fallback_pose_at <= 0.75):
            if not self.using_fallback_curve_pose:
                self.get_logger().warn(
                    'kR_in guide using /odometry/filtered fallback: '
                    'waiting for AMCL/P3D pose.')
                self.using_fallback_curve_pose = True
            return self.fallback_pose
        return None

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def kr_in_curve_assist(self, now, speed_hint):
        """Return a curvature from kR_in_k14 toward kR_in_k0, or None.

        The camera remains the normal lane reference.  This small map-frame
        pure-pursuit fallback is engaged only at the poorly painted turn, so
        a missing white segment cannot send the prototype into the barrier.
        """
        curve_pose = self.curve_pose_for_assist(now)
        if curve_pose is None:
            return None

        x, y, yaw = curve_pose
        # Trigger against the surveyed road junction, not the first inner
        # kerb sample.  This means the prototype follows the visible white
        # straight until the road genuinely starts to bend.
        entry_distance = math.hypot(
            x - self.kr_in_trigger_x, y - self.kr_in_trigger_y)

        if not self.kr_in_active:
            if entry_distance > self.kr_in_trigger_radius:
                return None
            first_x, first_y = self.kr_in_curve_points[0]
            next_x, next_y = self.kr_in_curve_points[1]
            direction_x = next_x - first_x
            direction_y = next_y - first_y
            direction_norm = math.hypot(direction_x, direction_y)
            heading_dot = (
                math.cos(yaw) * direction_x + math.sin(yaw) * direction_y
            ) / max(direction_norm, 1.0e-6)
            # The same markers are visible from the other side of the bend.
            # Enter only when the vehicle is travelling k14 -> k0.
            if heading_dot < 0.25:
                return None
            self.kr_in_active = True
            self.kr_in_index = 0
            self.get_logger().info(
                'kR_in curve assist engaged: joining the safe path parallel '
                'to kR_in_k14 -> kR_in_k0')

        distances = [math.hypot(x - px, y - py)
                     for px, py in self.kr_in_curve_points]
        nearest_index = min(range(len(distances)), key=distances.__getitem__)
        self.kr_in_index = max(self.kr_in_index, nearest_index)

        last_index = len(self.kr_in_curve_points) - 1
        if (self.kr_in_index >= last_index - 1 and
                distances[last_index] <= self.kr_in_exit_radius):
            self.kr_in_active = False
            self.get_logger().info('kR_in curve assist complete; lane keeping resumed')
            return None

        segment_index = min(self.kr_in_index, last_index - 1)
        segment_x, segment_y = self.kr_in_curve_points[segment_index]
        next_segment_x, next_segment_y = self.kr_in_curve_points[
            segment_index + 1]
        segment_dx = next_segment_x - segment_x
        segment_dy = next_segment_y - segment_y
        segment_length = max(math.hypot(segment_dx, segment_dy), 1.0e-6)
        tangent_x = segment_dx / segment_length
        tangent_y = segment_dy / segment_length
        path_heading = math.atan2(tangent_y, tangent_x)

        # Signed cross-track error: positive means the prototype is outside
        # (left of travel) and therefore must steer right toward kR_in.
        cross_track_error = (
            tangent_x * (y - segment_y) - tangent_y * (x - segment_x))
        heading_error = self.normalize_angle(path_heading - yaw)

        # Estimate the local reference curvature from adjacent segments.  It
        # is negative on this clockwise/right-hand bend and supplies turn
        # anticipation even before camera lane pixels become reliable.
        feedforward_curvature = 0.0
        if segment_index < last_index - 1:
            after_x, after_y = self.kr_in_curve_points[segment_index + 2]
            next_heading = math.atan2(
                after_y - next_segment_y, after_x - next_segment_x)
            feedforward_curvature = self.normalize_angle(
                next_heading - path_heading) / segment_length

        # Stanley feedback holds the parallel path.  The atan term stays
        # bounded when entering several metres outside the desired curve.
        safe_speed = max(0.7, abs(speed_hint))
        geometric_curvature = (
            feedforward_curvature +
            self.kr_in_heading_gain * heading_error -
            math.atan2(
                self.kr_in_cross_track_gain * cross_track_error, safe_speed))

        target_index = self.kr_in_index
        accumulated = 0.0
        while target_index < last_index and accumulated < self.kr_in_lookahead:
            x0, y0 = self.kr_in_curve_points[target_index]
            x1, y1 = self.kr_in_curve_points[target_index + 1]
            accumulated += math.hypot(x1 - x0, y1 - y0)
            target_index += 1

        target_x, target_y = self.kr_in_curve_points[target_index]
        dx = target_x - x
        dy = target_y - y
        lookahead_sq = dx * dx + dy * dy
        if lookahead_sq < 0.05:
            return 0.0

        # Pure-pursuit adds a smooth convergence term. Positive vehicle-frame
        # y is left, so this right turn produces negative curvature.
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        pure_pursuit_curvature = 2.0 * local_y / lookahead_sq
        curvature = (
            (1.0 - self.kr_in_pure_pursuit_weight) * geometric_curvature +
            self.kr_in_pure_pursuit_weight * pure_pursuit_curvature)

        # The white paint disappears near kR_in_k14.  While this dedicated
        # guide is active, retain a non-negotiable right-turn command even
        # if image detection is momentarily wrong.  The command is still
        # saturated by ``max_curvature`` (the real Ackermann limit) before
        # it reaches the steering joints.
        # kR_in_k14 is index 5 because the sequence is k19 -> k0.  Before
        # that point, follow the observed white lane/geometry normally; from
        # k14 onward retain the right-turn minimum so the camera cannot send
        # the vehicle straight into the outer barrier.
        k14_index = 5
        if self.kr_in_index >= k14_index:
            critical_progress = (self.kr_in_index - k14_index) / max(
                1, last_index - k14_index)
            forced_right_curvature = -0.24 * (
                1.0 + 0.25 * math.sin(critical_progress * math.pi))
            curvature = min(curvature, forced_right_curvature)

        self.curve_cross_track_pub.publish(
            Float32(data=float(cross_track_error)))
        self.curve_heading_pub.publish(Float32(data=float(heading_error)))
        self.curve_curvature_pub.publish(Float32(data=float(curvature)))
        return curvature

    def publish_command(self):
        now = time.monotonic()
        command = Twist()

        # Emergency Stop
        if self.stop_detected or self.stop_sign_detected:
            self.last_output_speed = 0.0
            self.last_output_at = now
            self.cmd_pub.publish(command)
            return

        lane_age = (now - self.last_lane_at) if self.last_lane_at > 0 else math.inf
        nav_fresh = (now - self.last_nav_command_at) <= self.command_timeout
        lane_active = (lane_age <= self.lane_timeout) and self.lane_valid

        # This map-frame guide deliberately runs before the vision-validity
        # branch: its purpose is to cross the kR_in turn even when its white
        # paint is invisible or ambiguous to the camera.
        speed_hint = (self.latest_nav_cmd.linear.x if nav_fresh else
                      self.kr_in_speed)
        kr_in_curvature = self.kr_in_curve_assist(now, speed_hint)
        if kr_in_curvature is not None:
            if nav_fresh:
                # Nav2 starts reducing speed before this tight bend.  The
                # dedicated kR_in geometry has priority here, so preserve its
                # safe 4.2 m/s crossing speed unless an external stop is set.
                curve_base_speed = max(
                    self.kr_in_speed, self.latest_nav_cmd.linear.x)
            else:
                curve_base_speed = self.target_speed
            if curve_base_speed > 0.05:
                curvature = max(-self.max_curvature, min(
                    self.max_curvature, kr_in_curvature))
                desired_speed = min(curve_base_speed, self.kr_in_speed)
                speed = max(self.min_curve_speed, desired_speed)
                command.linear.x = self.limit_speed(speed, now)
                command.angular.z = command.linear.x * curvature
                self.cmd_pub.publish(command)
                return

        # Case 1: Lane Keeping Active (with or without Nav2)
        if lane_active:
            # Determine base speed
            if nav_fresh and self.latest_nav_cmd.linear.x > 0.05:
                # Nav2 is actively navigating: follow its speed intent
                # With a valid lane, maintain the requested cruise speed.
                # Nav2 can still steer the route, while STOP safety remains
                # downstream and can always override this command.
                raw_speed = min(
                    self.max_speed,
                    max(self.target_speed, self.latest_nav_cmd.linear.x))
                base_curvature = self.latest_nav_cmd.angular.z / max(raw_speed, 0.1)
            else:
                # Nav2 idle or stopped at goal
                if nav_fresh and abs(self.latest_nav_cmd.linear.x) <= 0.05:
                    # Explicit Nav2 stop command (e.g. goal reached): honor stop
                    raw_speed = 0.0
                    base_curvature = 0.0
                else:
                    # Autonomous lane cruise
                    raw_speed = self.target_speed
                    base_curvature = 0.0

            if raw_speed <= 0.05:
                command.linear.x = self.limit_speed(0.0, now)
                command.angular.z = 0.0
                self.cmd_pub.publish(command)
                return

            # Compute lateral steering correction
            # Note sign convention:
            # lane_error > 0 means lane center is to the RIGHT of car -> car must steer RIGHT (negative angular.z)
            # lane_error < 0 means lane center is to the LEFT of car -> car must steer LEFT (positive angular.z)
            d_error = (self.lane_error - self.previous_lane_error)
            heading_gain = self.kh
            entering_right_curve = (
                self.heading_error > self.right_curve_threshold or
                self.lane_error > 0.04
            )
            if entering_right_curve:
                # Positive image error means the lane moves to the right;
                # ROS yaw must consequently be negative.  Use the near-field
                # lane as the stable reference and add anticipation only when
                # its far preview confirms a right-hand curve.
                heading_gain *= self.right_curve_gain
            correction = - (
                self.kp * self.lane_error + self.kd * d_error +
                heading_gain * self.heading_error
            )
            # In curves or off-center, strongly prioritize lane centering and curve navigation
            if abs(self.lane_error) > 0.08 or abs(self.heading_error) > 0.06:
                blend_weight = min(1.0, max(abs(self.lane_error) / 0.20, abs(self.heading_error) / 0.15))
                # Add a boost when turning into sharp bends
                curve_boost = 1.20 if (abs(self.heading_error) > 0.12 or abs(self.lane_error) > 0.15) else 1.0
                curvature = (1.0 - blend_weight) * base_curvature + blend_weight * (correction * curve_boost)
            else:
                curvature = 0.35 * base_curvature + 0.65 * correction


            # Bound curvature to vehicle physical Ackermann limit
            curvature = max(-self.max_curvature, min(self.max_curvature, curvature))

            # Dynamic speed regulation on bends: proactive deceleration into corners
            curve_ratio = max(abs(curvature) / self.max_curvature, abs(self.heading_error) / 0.25)
            off_center_ratio = min(1.0, abs(self.lane_error) / 0.25)
            speed_factor = max(
                0.55, 1.0 - 0.40 * curve_ratio - 0.25 * off_center_ratio)
            speed = max(self.min_curve_speed, min(raw_speed * speed_factor, self.max_speed))

            command.linear.x = self.limit_speed(speed, now)
            command.angular.z = command.linear.x * curvature

        # Case 2: Perception temporarily degraded -> Fall back smoothly to Nav2
        elif nav_fresh:
            command = self.latest_nav_cmd
            command.linear.x = self.limit_speed(min(max(command.linear.x, 0.0), self.max_speed), now)

        # Case 3: Neither Lane nor Nav2 available -> Controlled stop
        else:
            command.linear.x = self.limit_speed(0.0, now)
            command.angular.z = 0.0

        self.cmd_pub.publish(command)

    def limit_speed(self, target_speed: float, now: float) -> float:
        """Enforce acceleration and deceleration boundaries smoothly."""
        dt = max(0.001, now - self.last_output_at)
        if target_speed >= self.last_output_speed:
            allowed_delta = self.max_acceleration * dt
            speed = min(target_speed, self.last_output_speed + allowed_delta)
        else:
            allowed_delta = self.max_deceleration * dt
            speed = max(target_speed, self.last_output_speed - allowed_delta)

        self.last_output_speed = speed
        self.last_output_at = now
        return speed


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneKeepingController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
