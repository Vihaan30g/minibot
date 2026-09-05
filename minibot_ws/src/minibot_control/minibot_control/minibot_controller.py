"""
Minibot differential drive controller ROS 2 node.

Subscribes to high-level /cmd_vel (geometry_msgs/Twist) and publishes target
wheel angular velocities on /wheel_cmd_vel (std_msgs/Float32MultiArray) at a
deterministic update rate. Includes a built-in safety watchdog.
"""

import threading
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray

from minibot_control.kinematics import DifferentialKinematics


class MinibotController(Node):
    """ROS 2 Node responsible for differential drive inverse kinematics processing."""

    def __init__(self) -> None:
        """Initialize parameters, subscribers, publishers, timers, and internal state."""
        super().__init__('minibot_controller')

        # ------------------------------------------------------------------
        # Declare Parameters with Production Defaults
        # ------------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.339)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)

        # Fetch Parameter Values
        self.wheel_radius = (
            self.get_parameter('wheel_radius').get_parameter_value().double_value
        )
        self.wheel_base = (
            self.get_parameter('wheel_base').get_parameter_value().double_value
        )
        self.publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self.cmd_timeout = (
            self.get_parameter('cmd_timeout').get_parameter_value().double_value
        )
        self.max_linear_velocity = (
            self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        )
        self.max_angular_velocity = (
            self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        )

        # ------------------------------------------------------------------
        # Kinematics Engine Initialization
        # ------------------------------------------------------------------
        self.kinematics = DifferentialKinematics(
            wheel_radius=self.wheel_radius,
            wheel_base=self.wheel_base,
            max_linear_velocity=self.max_linear_velocity,
            max_angular_velocity=self.max_angular_velocity,
        )

        # ------------------------------------------------------------------
        # Thread Safety & State Variables
        # ------------------------------------------------------------------
        self._lock = threading.Lock()
        self._latest_twist: Twist = Twist()
        self._last_cmd_time: Optional[rclpy.time.Time] = None

        # ------------------------------------------------------------------
        # ROS 2 Interfaces (Pub, Sub, Timer)
        # ------------------------------------------------------------------
        # Velocity commands are perishable: only the newest value is ever
        # meaningful, so BEST_EFFORT + KEEP_LAST depth=1 means a momentary
        # stall anywhere in the chain (this node, the micro-ROS agent, the
        # serial link, the ESP32) drops stale commands instead of queuing
        # them for guaranteed in-order delivery. A reliable queue here would
        # let a brief hiccup "bank" delay that never gets undone, which
        # compounds over time into exactly the growing command-to-motion
        # lag this system was seeing. This QoS must be mirrored on the
        # ESP32 firmware's /wheel_cmd_vel subscriber
        # (rclc_subscription_init_best_effort), which is the other end of
        # this same command path.
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            cmd_qos
        )

        self._wheel_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_cmd_vel',
            cmd_qos
        )

        timer_period = 1.0 / self.publish_rate
        self._control_timer = self.create_timer(timer_period, self._control_loop_callback)

        # Print Startup Diagnostic Info
        self.get_logger().info('====================================================')
        self.get_logger().info('Minibot Controller Node Started Successfully')
        self.get_logger().info(f'  Wheel Radius        : {self.wheel_radius:.3f} m')
        self.get_logger().info(f'  Wheel Base          : {self.wheel_base:.3f} m')
        self.get_logger().info(f'  Publish Rate        : {self.publish_rate:.1f} Hz')
        self.get_logger().info(f'  Command Timeout     : {self.cmd_timeout:.2f} s')
        self.get_logger().info(f'  Max Linear Velocity : {self.max_linear_velocity:.2f} m/s')
        self.get_logger().info(f'  Max Angular Velocity: {self.max_angular_velocity:.2f} rad/s')
        self.get_logger().info('====================================================')

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Asynchronous subscriber callback for incoming /cmd_vel messages.

        Stores the latest command and records arrival timestamp under lock protection.

        :param msg: Incoming geometry_msgs/Twist velocity command.
        """
        with self._lock:
            self._latest_twist = msg
            self._last_cmd_time = self.get_clock().now()

    def _control_loop_callback(self) -> None:
        """
        Deterministic timer-driven loop operating at configured publish_rate.

        Evaluates the watchdog timeout condition, processes inverse kinematics,
        and publishes left/right wheel velocity commands.
        """
        current_time = self.get_clock().now()

        with self._lock:
            last_time = self._last_cmd_time
            twist_cmd = self._latest_twist

        # ------------------------------------------------------------------
        # Safety Watchdog Verification
        # ------------------------------------------------------------------
        is_timed_out = False
        if last_time is None:
            is_timed_out = True
        else:
            time_since_last_cmd = (current_time - last_time).nanoseconds / 1e9
            if time_since_last_cmd > self.cmd_timeout:
                is_timed_out = True

        # ------------------------------------------------------------------
        # Velocity Computation & Command Assembly
        # ------------------------------------------------------------------
        if is_timed_out:
            left_rad_s = 0.0
            right_rad_s = 0.0
        else:
            left_rad_s, right_rad_s = self.kinematics.twist_to_wheel_velocities(
                twist_cmd.linear.x,
                twist_cmd.angular.z
            )

        # Prepare std_msgs/Float32MultiArray output packet
        # Index 0: Left wheel angular velocity (rad/s)
        # Index 1: Right wheel angular velocity (rad/s)
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [float(left_rad_s), float(right_rad_s)]

        self._wheel_cmd_pub.publish(cmd_msg)


def main(args: list = None) -> None:
    """Node entry point."""
    rclpy.init(args=args)
    node = MinibotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Minibot Controller shutting down gracefully...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
