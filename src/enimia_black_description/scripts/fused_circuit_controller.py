#!/usr/bin/env python3
"""
Fused Circuit Controller & Closed-Loop Lap Manager for ENIMIA Black.
Tuned for Perfect Hairpin Negotiation & High-Speed Circuit Racing:
1. Dynamic Speed Profiling:
   - Full power on straights: 7.5 m/s (~27 km/h).
   - Proactive braking before & inside tight hairpins (down to 3.0 m/s) to eliminate understeer.
   - Rapid acceleration back to 7.5 m/s on corner exit.
2. High-Authority Steering in Curves:
   - Maximum steering angle expanded to 0.45 rad (~26 deg) to execute tight 180-deg hairpins effortlessly.
   - Curvature feedforward gain kc=0.80 (up to 0.25 rad) perfectly matching the track radius.
   - Slew-rate limiter at 1.20 rad/s for fast yet smooth wheel movement.
3. Active LiDAR Barrier Repulsion:
   - Repels car from outer and inner walls (safe margin 2.20 m).
   - Emergency brake on obstacle < 1.30 m.
4. Mandatory 5-Second Stop on yellow stop line & STOP signs.
"""

import math
import time
import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class FusedCircuitController(Node):
    # Circuit States
    STATE_START = "START_GRID"
    STATE_CIRCUIT_CRUISE = "CIRCUIT_CRUISE"
    STATE_STOP_HOLD = "STOP_HOLD_5S"
    STATE_APPROACH_FINISH = "APPROACH_FINISH"
    STATE_LAP_COMPLETED = "LAP_COMPLETED_STOP"

    def __init__(self):
        super().__init__('fused_circuit_controller')

        # Topics
        self.declare_parameter('input_cmd_topic', '/cmd_vel_safe')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_lane')
        self.declare_parameter('lane_error_topic', '/lane_detection/center_error')
        self.declare_parameter('heading_error_topic', '/lane_detection/heading_error')
        self.declare_parameter('curvature_topic', '/lane_detection/curvature')
        self.declare_parameter('lane_valid_topic', '/lane_detection/valid')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('stop_topic', '/stop_line/detected')
        self.declare_parameter('stop_sign_topic', '/stop_sign/detected')
        self.declare_parameter('lidar_bias_topic', '/lidar/barrier_steer_bias')
        self.declare_parameter('lidar_brake_topic', '/lidar/emergency_brake')

        # Vehicle Kinematics
        self.declare_parameter('wheelbase', 1.704)
        self.declare_parameter('max_steering_angle', 0.45)   # Full Ackermann envelope (26 deg) for tight hairpins
        self.declare_parameter('max_steer_rate', 1.20)       # rad/s steering rate limit

        # Speed Profile (Fast on straights, crisp grip in hairpins)
        self.declare_parameter('cruise_speed', 7.5)          # Straight line cruise (m/s ~ 27 km/h)
        self.declare_parameter('start_speed', 5.0)           # Initial launch speed (m/s)
        self.declare_parameter('min_curve_speed', 3.0)       # Hairpin apex speed (m/s ~ 11 km/h - zero tire slip!)
        self.declare_parameter('max_speed', 8.2)             # Max allowable velocity
        self.declare_parameter('max_acceleration', 6.0)      # Fast throttle response
        self.declare_parameter('max_deceleration', 10.0)

        # Control Gains
        self.declare_parameter('kp', 1.15)                   # Strong lateral centering feedback
        self.declare_parameter('kd', 0.18)                   # Derivative damping
        self.declare_parameter('kh', 0.35)                   # Heading tracking into curve
        self.declare_parameter('kc', 0.80)                   # Curvature feedforward (matches track radius)
        self.declare_parameter('max_curve_feedforward', 0.25) # Max ~14.5 deg curve feedforward
        self.declare_parameter('stop_duration', 5.0)

        # Dynamic Lookahead Parameters
        self.declare_parameter('min_lookahead', 2.8)
        self.declare_parameter('max_lookahead', 5.5)

        # Circuit Lap Completion Parameters
        self.declare_parameter('min_lap_distance', 150.0)
        self.declare_parameter('finish_radius', 4.5)
        self.declare_parameter('enable_circuit_lap_stop', True)

        self.declare_parameter('lane_timeout', 0.8)
        self.declare_parameter('publish_rate', 40.0)

        # Unpack parameters
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.lane_error_topic = self.get_parameter('lane_error_topic').value
        self.heading_error_topic = self.get_parameter('heading_error_topic').value
        self.curvature_topic = self.get_parameter('curvature_topic').value
        self.lane_valid_topic = self.get_parameter('lane_valid_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.stop_sign_topic = self.get_parameter('stop_sign_topic').value
        self.lidar_bias_topic = self.get_parameter('lidar_bias_topic').value
        self.lidar_brake_topic = self.get_parameter('lidar_brake_topic').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.max_steer_rate = float(self.get_parameter('max_steer_rate').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.start_speed = float(self.get_parameter('start_speed').value)
        self.min_curve_speed = float(self.get_parameter('min_curve_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.max_deceleration = float(self.get_parameter('max_deceleration').value)

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.kh = float(self.get_parameter('kh').value)
        self.kc = float(self.get_parameter('kc').value)
        self.max_curve_ff = float(self.get_parameter('max_curve_feedforward').value)
        self.stop_duration = float(self.get_parameter('stop_duration').value)

        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)
        self.min_lap_distance = float(self.get_parameter('min_lap_distance').value)
        self.finish_radius = float(self.get_parameter('finish_radius').value)
        self.enable_circuit_lap_stop = bool(self.get_parameter('enable_circuit_lap_stop').value)
        self.lane_timeout = float(self.get_parameter('lane_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # State tracking
        self.circuit_state = self.STATE_START
        self.start_pose = None
        self.current_pose = None
        self.last_pose = None
        self.cumulative_distance = 0.0

        self.lane_error = 0.0
        self.previous_lane_error = 0.0
        self.heading_error = 0.0
        self.curvature = 0.0
        self.lane_valid = False
        self.last_lane_at = 0.0

        # LiDAR obstacle & barrier feedback
        self.lidar_barrier_bias = 0.0
        self.lidar_emergency_brake = False

        # 5-second Stop Line & Stop Sign Hold State
        self.stop_line_detected = False
        self.stop_sign_detected = False
        self.stop_hold_start = None
        self.stop_cleared_after_hold = False

        self.latest_nav_cmd = Twist()
        self.last_nav_at = 0.0

        self.last_output_speed = 0.0
        self.last_output_steer = 0.0
        self.last_loop_time = time.monotonic()
        self.lap_finished_time = None
        self.start_time = time.monotonic()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.state_pub = self.create_publisher(String, '/circuit/lap_state', 10)
        self.dist_pub = self.create_publisher(Float32, '/circuit/lap_distance', 10)
        self.completed_pub = self.create_publisher(Bool, '/circuit/lap_completed', 10)

        # Subscriptions
        self.create_subscription(Twist, self.input_cmd_topic, self.nav_cmd_callback, 10)
        self.create_subscription(Float32, self.lane_error_topic, self.lane_error_callback, 10)
        self.create_subscription(Float32, self.heading_error_topic, self.heading_error_callback, 10)
        self.create_subscription(Float32, self.curvature_topic, self.curvature_callback, 10)
        self.create_subscription(Bool, self.lane_valid_topic, self.lane_valid_callback, 10)
        self.create_subscription(Bool, self.stop_topic, self.stop_callback, 10)
        self.create_subscription(Bool, self.stop_sign_topic, self.stop_sign_callback, 10)
        self.create_subscription(Float32, self.lidar_bias_topic, self.lidar_bias_callback, 10)
        self.create_subscription(Bool, self.lidar_brake_topic, self.lidar_brake_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        if self.odom_topic != '/odom':
            self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(1.0 / publish_rate, self.control_loop)

        self.get_logger().info(
            f'Fused Circuit Controller Active (Fast Straights: {self.cruise_speed}m/s | Hairpins: {self.min_curve_speed}m/s, Steer: {self.max_steering_angle}rad)'
        )

    def nav_cmd_callback(self, msg: Twist):
        self.latest_nav_cmd = msg
        self.last_nav_at = time.monotonic()

    def lane_error_callback(self, msg: Float32):
        self.previous_lane_error = self.lane_error
        self.lane_error = float(max(-1.0, min(1.0, msg.data)))

    def heading_error_callback(self, msg: Float32):
        self.heading_error = float(max(-1.0, min(1.0, msg.data)))

    def curvature_callback(self, msg: Float32):
        self.curvature = float(max(-0.5, min(0.5, msg.data)))

    def lane_valid_callback(self, msg: Bool):
        self.lane_valid = bool(msg.data)
        if self.lane_valid:
            self.last_lane_at = time.monotonic()

    def stop_callback(self, msg: Bool):
        self.stop_line_detected = bool(msg.data)

    def stop_sign_callback(self, msg: Bool):
        self.stop_sign_detected = bool(msg.data)

    def lidar_bias_callback(self, msg: Float32):
        self.lidar_barrier_bias = float(msg.data)

    def lidar_brake_callback(self, msg: Bool):
        self.lidar_emergency_brake = bool(msg.data)

    def odom_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        if self.start_pose is None:
            self.start_pose = (px, py)
            self.last_pose = (px, py)
            self.get_logger().info(f'Start Grid Pose: x0={px:.2f}, y0={py:.2f}')

        self.current_pose = (px, py)

        if self.last_pose is not None:
            dx = px - self.last_pose[0]
            dy = py - self.last_pose[1]
            ds = math.hypot(dx, dy)
            if ds < 10.0:
                self.cumulative_distance += ds
            else:
                self.start_time = time.monotonic()
                self.get_logger().info('Véhicule repositionné : réinitialisation départ plein régime !')
        self.last_pose = (px, py)

    def control_loop(self):
        now = time.monotonic()
        dt = max(0.001, min(0.1, now - self.last_loop_time))
        self.last_loop_time = now

        command = Twist()

        # Update circuit lap state
        self.update_lap_state()

        # Telemetry
        self.state_pub.publish(String(data=self.circuit_state))
        self.dist_pub.publish(Float32(data=float(self.cumulative_distance)))
        self.completed_pub.publish(Bool(data=(self.circuit_state == self.STATE_LAP_COMPLETED)))

        # Case 1: Full Lap Completed -> Standstill on finish line
        if self.circuit_state == self.STATE_LAP_COMPLETED:
            command.linear.x = self.smooth_speed(0.0, dt)
            command.angular.z = 0.0
            self.cmd_pub.publish(command)
            return

        # Case 2: 5-Second Mandatory Stop (Yellow Stop Line or STOP Sign)
        stop_triggered = self.stop_line_detected or self.stop_sign_detected
        if stop_triggered and not self.stop_cleared_after_hold:
            if self.stop_hold_start is None:
                self.stop_hold_start = now
                self.get_logger().warn('🛑 ARRET OBLIGATOIRE (LIGNE JAUNE / STOP SIGN) : Début du compte à rebours 5s !')

            elapsed_hold = now - self.stop_hold_start
            remaining = max(0.0, self.stop_duration - elapsed_hold)

            if remaining > 0.0:
                self.get_logger().info(f'CHRONO ARRET : {remaining:.1f} s restantes / {self.stop_duration:.1f} s')
                command.linear.x = self.smooth_speed(0.0, dt)
                command.angular.z = 0.0
                self.cmd_pub.publish(command)
                return
            else:
                self.get_logger().info('✅ 5 SECONDES ECOULEES : Redémarrage propre du véhicule !')
                self.stop_cleared_after_hold = True
                self.stop_hold_start = None

        if not stop_triggered:
            self.stop_cleared_after_hold = False
            self.stop_hold_start = None

        # Case 3: LiDAR Imminent Collision Emergency Brake
        if self.lidar_emergency_brake:
            self.get_logger().warn('⚠️ FREINAGE D URGENCE LIDAR (Obstacle / Barrière trop proche !)')
            command.linear.x = self.smooth_speed(0.0, dt)
            command.angular.z = 0.0
            self.cmd_pub.publish(command)
            return

        lane_age = now - self.last_lane_at if self.last_lane_at > 0 else math.inf
        lane_active = (lane_age <= self.lane_timeout) and self.lane_valid

        # Case 4: Active Racing & Smooth Hairpin Curve Navigation
        if lane_active:
            # 1. Progressive Curvature-Adaptive Speed Regulation:
            # Fast straights: 7.5 m/s (~27 km/h)
            # Hairpin turns: smooth proactive brake down to 3.0 m/s (~11 km/h) for 100% adherence!
            elapsed_from_start = now - self.start_time
            if elapsed_from_start < 0.6:
                speed_target = self.start_speed
            else:
                # curve_factor measures how sharp the curve is (0.0 on straight, 1.0 in hairpin)
                curve_factor = min(1.0, max(abs(self.curvature) / 0.08, abs(self.heading_error) / 0.14))
                error_factor = min(0.3, abs(self.lane_error) / 0.50)
                speed_target = self.cruise_speed - curve_factor * (self.cruise_speed - self.min_curve_speed) - error_factor * 2.0
                speed_target = max(self.min_curve_speed, min(speed_target, self.max_speed))

            # Approach finish line
            if self.circuit_state == self.STATE_APPROACH_FINISH and self.start_pose and self.current_pose:
                dist_to_start = math.hypot(self.current_pose[0] - self.start_pose[0],
                                           self.current_pose[1] - self.start_pose[1])
                finish_taper = max(0.5, min(1.0, dist_to_start / (self.finish_radius * 2.0)))
                speed_target = max(2.8, speed_target * finish_taper)

            # 2. High-Authority Hairpin Steering Calculation:
            # Sign convention:
            # lane_error > 0 => car is left of center -> steer right (negative angular.z)
            # lane_error < 0 => car is right of center -> steer left (positive angular.z)
            d_error = (self.lane_error - self.previous_lane_error) / dt
            feedback_steer = -(self.kp * self.lane_error + self.kd * d_error)

            # Heading anticipation into the curve:
            heading_term = -self.kh * self.heading_error

            # Curvature feedforward:
            theoretical_curve_steer = -math.atan(self.wheelbase * self.curvature)
            clamped_curve_ff = float(np.clip(self.kc * theoretical_curve_steer, -self.max_curve_ff, self.max_curve_ff))

            # Combine control terms + active LiDAR barrier repulsion
            target_steer = feedback_steer + heading_term + clamped_curve_ff + self.lidar_barrier_bias
            target_steer = float(np.clip(target_steer, -self.max_steering_angle, self.max_steering_angle))

            # 3. Fast & Smooth Steering Slew-Rate Limiter:
            max_delta = self.max_steer_rate * dt
            steer_diff = target_steer - self.last_output_steer
            clamped_steer = self.last_output_steer + float(np.clip(steer_diff, -max_delta, max_delta))
            clamped_steer = float(np.clip(clamped_steer, -self.max_steering_angle, self.max_steering_angle))

            # Apply commands
            command.linear.x = self.smooth_speed(speed_target, dt)
            command.angular.z = command.linear.x * math.tan(clamped_steer) / self.wheelbase
            self.last_output_steer = clamped_steer

        # Case 5: Blind Apex -> Smooth decay at minimum curve speed
        elif lane_age <= (self.lane_timeout + 1.2):
            self.last_output_steer *= 0.92
            speed_target = self.min_curve_speed
            command.linear.x = self.smooth_speed(speed_target, dt)
            command.angular.z = command.linear.x * math.tan(self.last_output_steer) / self.wheelbase

        # Case 6: Lost -> Controlled stop
        else:
            command.linear.x = self.smooth_speed(0.0, dt)
            command.angular.z = 0.0

        self.cmd_pub.publish(command)

    def update_lap_state(self):
        """Monitors circuit progression and manages start-to-finish loop closure."""
        if self.start_pose is None or self.current_pose is None:
            return

        dist_from_start = math.hypot(self.current_pose[0] - self.start_pose[0],
                                     self.current_pose[1] - self.start_pose[1])

        if self.circuit_state == self.STATE_START:
            if self.cumulative_distance > 15.0:
                self.circuit_state = self.STATE_CIRCUIT_CRUISE
                self.get_logger().info('Circuit Navigation: En piste !')

        elif self.circuit_state == self.STATE_CIRCUIT_CRUISE:
            if self.cumulative_distance >= self.min_lap_distance:
                if dist_from_start < (self.finish_radius * 2.5):
                    self.circuit_state = self.STATE_APPROACH_FINISH
                    self.get_logger().info('Circuit Navigation: Ligne d arrivée en vue !')

        elif self.circuit_state == self.STATE_APPROACH_FINISH:
            if dist_from_start <= self.finish_radius:
                self.circuit_state = self.STATE_LAP_COMPLETED
                self.lap_finished_time = time.monotonic()
                self.get_logger().info(
                    f'🏁 TOUR COMPLET DU CIRCUIT TERMINE ! Distance: {self.cumulative_distance:.2f} m.'
                )

    def smooth_speed(self, target: float, dt: float) -> float:
        """Applies physical acceleration and braking ramps smoothly."""
        if target >= self.last_output_speed:
            allowed = self.max_acceleration * dt
            speed = min(target, self.last_output_speed + allowed)
        else:
            allowed = self.max_deceleration * dt
            speed = max(target, self.last_output_speed - allowed)

        self.last_output_speed = speed
        return speed


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FusedCircuitController()
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
