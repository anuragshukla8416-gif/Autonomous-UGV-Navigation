import unittest
import numpy as np
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers')))

from localization.wheel_odometry import WheelOdometry
from localization.imu_filter import ImuFilter
from localization.pose_estimator import PoseEstimator
from localization.visual_odometry import VisualOdometry

class TestLocalization(unittest.TestCase):
    def test_wheel_odometry(self):
        odom = WheelOdometry(track_width=0.46, wheel_radius=0.095)
        # First reading initializes
        odom.update(0.0, 0.0, 0.1)
        # Advance both wheels by 10 rad in 1 sec -> ds = 0.095 * 10 = 0.95m
        v, w, ds, dtheta = odom.update(10.0, 10.0, 1.0)
        self.assertAlmostEqual(v, 0.95, places=2)
        self.assertAlmostEqual(w, 0.0, places=2)
        self.assertAlmostEqual(ds, 0.95, places=2)

    def test_imu_unwrapping(self):
        filt = ImuFilter()
        filt.update([0, 0, 3.10], [0, 0, 0], 0.1)
        # Jump across pi boundary
        roll, pitch, yaw_unwrapped, wz = filt.update([0, 0, -3.10], [0, 0, 0], 0.1)
        # Delta should be +0.083, not -6.2
        self.assertGreater(yaw_unwrapped, 3.10)

    def test_pose_estimator_ekf(self):
        est = PoseEstimator(init_x=0.0, init_y=0.0, init_theta=0.0)
        # Drive forward at 0.5 m/s for 10 steps of 0.1s = 1.0s -> ~0.5m
        for _ in range(10):
            est.predict(0.1)
            est.update_wheel_odometry(0.5, 0.0)
            est.update_imu(0.0, 0.0)
        
        x, y, theta, v, w = est.get_pose()
        self.assertGreater(x, 0.35)
        self.assertAlmostEqual(y, 0.0, places=1)
        self.assertAlmostEqual(theta, 0.0, places=1)

    def test_visual_odometry_interface(self):
        vo = VisualOdometry()
        rgb = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        depth = np.ones((240, 320), dtype=np.float32) * 2.0
        dx, dy, dtheta, inliers = vo.process_frame(rgb, depth, 0.032)
        self.assertEqual(dx, 0.0) # first frame returns 0

if __name__ == '__main__':
    unittest.main()
