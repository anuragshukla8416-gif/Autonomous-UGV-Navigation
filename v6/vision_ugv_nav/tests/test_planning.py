import unittest
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers')))

from planning.a_star import AStarPlanner
from planning.theta_star import ThetaStarPlanner
from planning.dwa_planner import DWAPlanner
from planning.recovery_manager import RecoveryManager
from perception.traversability_map import TraversabilityMap
from control.diff_drive_controller import DiffDriveController

class TestPlanningAndControl(unittest.TestCase):
    def test_theta_star_planner(self):
        planner = ThetaStarPlanner(x_bounds=(-10.0, 10.0), y_bounds=(-10.0, 10.0), resolution=0.5)
        # 1. Straight line without obstacles should be short, direct
        path_direct = planner.plan((0.0, 0.0), (4.0, 0.0))
        self.assertGreater(len(path_direct), 1)
        self.assertAlmostEqual(path_direct[-1][0], 4.0, delta=0.5)
        self.assertAlmostEqual(path_direct[-1][1], 0.0, delta=0.5)

        # 2. Place obstacle directly between start and goal
        planner.set_obstacle(2.0, 0.0, radius=0.8)
        path_detour = planner.plan((0.0, 0.0), (4.0, 0.0))
        self.assertGreater(len(path_detour), 2)
        # Any-angle detour must avoid (2.0, 0.0)
        mid_pts = [pt for pt in path_detour if 1.0 <= pt[0] <= 3.0]
        self.assertTrue(any(abs(pt[1]) > 0.4 for pt in mid_pts))

    def test_a_star_planner(self):
        planner = AStarPlanner(x_bounds=(-10.0, 10.0), y_bounds=(-10.0, 10.0), resolution=0.5)
        # Place obstacle directly between start (0, 0) and goal (4, 0)
        planner.set_obstacle(2.0, 0.0, radius=0.8)
        path = planner.plan((0.0, 0.0), (4.0, 0.0))
        self.assertGreater(len(path), 2)
        # Path must detour around obstacle (y != 0 at x ~ 2)
        detours = [pt for pt in path if 1.0 <= pt[0] <= 3.0]
        self.assertTrue(any(abs(pt[1]) > 0.4 for pt in detours))

    def test_dwa_planner(self):
        dwa = DWAPlanner()
        tmap = TraversabilityMap(resolution=0.1, size_x=6.0, size_y=6.0)
        tmap.update(None) # Clear map
        
        # Target straight ahead at (5.0, 0.0)
        v, w, best_traj, all_trajs = dwa.plan(
            0.0, 0.0, (0.0, 0.0, 0.0), (5.0, 0.0), tmap
        )
        self.assertGreater(v, 0.1)
        self.assertAlmostEqual(w, 0.0, delta=0.2)

    def test_recovery_manager(self):
        rec = RecoveryManager(stuck_time_threshold=1.0, check_interval=0.5)
        rec.update((0.0, 0.0), 0.0, 0.1)
        # Check after 0.6s -> sets stuck_start_time = 0.6
        rec.update((0.0, 0.0), 0.6, 0.1)
        # Check after 1.8s (1.8 - 0.6 = 1.2s stuck) -> triggers recovery
        is_rec, v, w, replan = rec.update((0.0, 0.0), 1.8, 0.1)
        self.assertTrue(is_rec)
        self.assertLess(v, 0.0) # Reversing

    def test_diff_drive_controller(self):
        ctrl = DiffDriveController(None, None, track_width=0.46, wheel_radius=0.095)
        applied_v, applied_w, wl, wr = ctrl.set_cmd_vel(0.5, 0.0, dt=0.5)
        self.assertGreater(applied_v, 0.1)
        self.assertAlmostEqual(wl, wr, places=2)

if __name__ == '__main__':
    unittest.main()
