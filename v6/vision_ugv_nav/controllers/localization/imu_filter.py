import math
import numpy as np

class ImuFilter:
    """
    Processes InertialUnit (RPY) and Gyroscope angular rates.
    Performs yaw un-wrapping and gyro bias correction.
    """
    def __init__(self):
        self.yaw_prev = None
        self.yaw_unwrapped = 0.0

    def update(self, rpy_values, gyro_values, dt):
        """
        Args:
            rpy_values: [roll, pitch, yaw] from InertialUnit in radians.
            gyro_values: [wx, wy, wz] angular rates in rad/s.
            dt: time step in seconds.
        Returns:
            roll, pitch, yaw_unwrapped, wz
        """
        if rpy_values is None:
            return 0.0, 0.0, self.yaw_unwrapped, 0.0

        roll, pitch, yaw = rpy_values[0], rpy_values[1], rpy_values[2]
        wz = gyro_values[2] if gyro_values is not None else 0.0

        if self.yaw_prev is None:
            self.yaw_prev = yaw
            self.yaw_unwrapped = yaw
            return roll, pitch, yaw, wz

        # Un-wrap angular delta across [-pi, pi]
        dyaw = yaw - self.yaw_prev
        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        self.yaw_unwrapped += dyaw
        self.yaw_prev = yaw

        return roll, pitch, self.yaw_unwrapped, wz
