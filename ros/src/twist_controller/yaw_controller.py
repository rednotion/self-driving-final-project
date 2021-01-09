"""
A controller that can be used to convert target linear and angular velocity to steering commands.
"""

from math import atan


class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        lin_vel_greater_than_zero = True if (abs(linear_velocity.x) > 0) & (abs(linear_velocity.y) > 0) & (abs(linear_velocity.z) > 0) else False
        ang_vel_greater_than_zero = True if (abs(angular_velocity.x) > 0) & (abs(angular_velocity.y) > 0) & (abs(angular_velocity.z) > 0) else False

        angular_velocity = current_velocity * angular_velocity / linear_velocity if lin_vel_greater_than_zero else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity)
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if ang_vel_greater_than_zero > 0. else 0.0
