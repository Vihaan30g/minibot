#!/usr/bin/env python3
"""
wheel_odometry_node.py

Subscribes to /wheel_states (sensor_msgs/JointState) published by the
ESP32 micro-ROS firmware and computes + publishes wheel odometry.

Firmware publishes at 50 Hz (UROS_PUBLISH_FREQUENCY).
Position is cumulative radians, already direction-corrected.
Velocity is rad/s, already direction-corrected.

Topic subscribed : wheel_states   (sensor_msgs/JointState)
Topic published  : wheel/odom     (nav_msgs/Odometry)
TF broadcast     : odom → base_link 
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        # ── Robot physical parameters ─────────────────────────────────
        # Measure these precisely on your actual robot.
        self.WHEEL_RADIUS = 0.04615   
        self.WHEEL_BASE   = 0.27692    # metres  (centre-to-centre of drive wheels)

        # ── Pose state ────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Previous wheel positions in radians — seeded on first message
        self.prev_left_pos  = None   # radians
        self.prev_right_pos = None

        # ── QoS matching firmware's best-effort publisher ─────────────
        # Firmware uses rclc_publisher_init_best_effort → must match here
        # or ROS2 will silently drop all messages.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publisher ─────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(
            Odometry, '/wheel/odom', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber ────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            'wheel_states',       # exact topic name from firmware UROS_STATE_TOPIC
            self.joint_state_callback,
            qos)

        self.get_logger().info('Wheel odometry node started — waiting for wheel_states...')


    def joint_state_callback(self, msg: JointState):

        # ── Validate message has what we need ─────────────────────────
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            self.get_logger().warn(
                'wheel_states message has insufficient data', throttle_duration_sec=5.0)
            return

        curr_left_pos  = msg.position[0]   # radians, cumulative
        curr_right_pos = msg.position[1]   # radians, cumulative

        curr_left_vel  = msg.velocity[0]   # rad/s
        curr_right_vel = msg.velocity[1]   # rad/s

        # ── Seed on first message — no odom to compute yet ────────────
        if self.prev_left_pos is None:
            self.prev_left_pos  = curr_left_pos
            self.prev_right_pos = curr_right_pos
            self.get_logger().info('First wheel_states received — odometry initialised.')
            return

        # ── Delta wheel angle since last callback (radians) ───────────
        delta_left_rad  = curr_left_pos  - self.prev_left_pos
        delta_right_rad = curr_right_pos - self.prev_right_pos

        self.prev_left_pos  = curr_left_pos
        self.prev_right_pos = curr_right_pos

        # ── Wheel arc length (metres) ─────────────────────────────────
        delta_left_m  = delta_left_rad  * self.WHEEL_RADIUS
        delta_right_m = delta_right_rad * self.WHEEL_RADIUS

        # ── Differential drive kinematics ─────────────────────────────
        delta_s     = (delta_right_m + delta_left_m)  / 2.0   # forward displacement
        delta_theta = (delta_right_m - delta_left_m)  / self.WHEEL_BASE  # heading change

        # ── Integrate pose (midpoint / second-order Runge-Kutta) ──────
        # More accurate than Euler for curved paths.
        mid_theta   = self.theta + delta_theta / 2.0
        self.x     += delta_s * math.cos(mid_theta)
        self.y     += delta_s * math.sin(mid_theta)
        self.theta += delta_theta

        # Normalise theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ── Linear and angular velocity from current wheel velocities ──
        # Use velocity fields directly — smoother than delta_s/dt because
        # the firmware already computed these at 1000 Hz and the 50 Hz
        # publish averages out noise better than our single-step delta.
        left_linear  = curr_left_vel  * self.WHEEL_RADIUS   # m/s
        right_linear = curr_right_vel * self.WHEEL_RADIUS   # m/s

        vx     = (right_linear + left_linear)  / 2.0        # m/s forward
        vtheta = (right_linear - left_linear)  / self.WHEEL_BASE  # rad/s

        # ── Stamp — use firmware header stamp if available, else now ──
        if msg.header.stamp.sec != 0:
            stamp = msg.header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # ── Build Odometry message ────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Yaw → quaternion (flat ground, only yaw changes)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Twist (in base_link frame)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = vtheta

        # ── Covariance ────────────────────────────────────────────────
        # Row-major 6x6 matrix indices for [x, y, z, roll, pitch, yaw]
        # Pose — don't trust absolute wheel position too strongly;
        # EKF will anchor it via visual odometry from ZED2i.
        odom.pose.covariance[0]  = 0.05    # x
        odom.pose.covariance[7]  = 0.05    # y
        odom.pose.covariance[14] = 1e9     # z  (irrelevant, flat floor)
        odom.pose.covariance[21] = 1e9     # roll  (irrelevant)
        odom.pose.covariance[28] = 1e9     # pitch (irrelevant)
        odom.pose.covariance[35] = 0.1     # yaw

        # Twist — velocity is more trustworthy than integrated position
        odom.twist.covariance[0]  = 0.01   # vx
        odom.twist.covariance[7]  = 1e9    # vy  (non-holonomic — always ~0)
        odom.twist.covariance[14] = 1e9    # vz
        odom.twist.covariance[21] = 1e9    # vroll
        odom.twist.covariance[28] = 1e9    # vpitch
        odom.twist.covariance[35] = 0.05   # vyaw

        self.odom_pub.publish(odom)

        # ── TF broadcast: odom → base_link ────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()