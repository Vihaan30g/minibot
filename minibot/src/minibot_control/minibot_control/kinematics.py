"""
Kinematics and velocity limiting utilities for Minibot differential drive robot.

This module provides pure functional and object-oriented interfaces for
forward and inverse differential drive kinematics, velocity saturation, and
conversion math without ROS 2 node dependencies.
"""

from typing import Tuple


class DifferentialKinematics:
    """Handles differential drive kinematics calculations and velocity saturation."""

    def __init__(
        self,
        wheel_radius: float,
        wheel_base: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
    ) -> None:
        """
        Initialize differential drive kinematics configuration.

        :param wheel_radius: Radius of the drive wheels in meters.
        :param wheel_base: Center-to-center distance between left and right wheels in meters.
        :param max_linear_velocity: Maximum absolute linear velocity in m/s.
        :param max_angular_velocity: Maximum absolute angular velocity in rad/s.
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

    def clamp_velocities(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        """
        Saturate linear and angular velocities to configured maximum bounds.

        :param linear_x: Unconstrained linear velocity command (m/s).
        :param angular_z: Unconstrained angular velocity command (rad/s).
        :return: Saturation-constrained tuple of (linear_x, angular_z).
        """
        clamped_linear = max(
            -self.max_linear_velocity,
            min(self.max_linear_velocity, linear_x)
        )
        clamped_angular = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, angular_z)
        )
        return clamped_linear, clamped_angular

    def twist_to_wheel_velocities(
        self, linear_x: float, angular_z: float
    ) -> Tuple[float, float]:
        """
        Convert body-frame Twist velocities into left and right wheel angular velocities.

        Differential Drive Inverse Kinematics Mathematics:
        -------------------------------------------------
        1. Compute target linear speeds at left and right wheel contacts:
           v_left  = v_linear - (w_angular * wheel_base / 2)
           v_right = v_linear + (w_angular * wheel_base / 2)

        2. Convert tangential wheel velocities (m/s) to wheel angular velocities (rad/s):
           omega_left  = v_left / wheel_radius
           omega_right = v_right / wheel_radius

        :param linear_x: Target body linear velocity (m/s).
        :param angular_z: Target body angular velocity (rad/s).
        :return: Tuple of (left_wheel_rad_s, right_wheel_rad_s).
        """
        sat_linear, sat_angular = self.clamp_velocities(linear_x, angular_z)

        # Calculate wheel tangential linear velocities (m/s)
        left_linear_m_s = sat_linear - (sat_angular * self.wheel_base / 2.0)
        right_linear_m_s = sat_linear + (sat_angular * self.wheel_base / 2.0)

        # Convert tangential linear velocities to rotational angular velocities (rad/s)
        left_rad_s = left_linear_m_s / self.wheel_radius
        right_rad_s = right_linear_m_s / self.wheel_radius

        return left_rad_s, right_rad_s