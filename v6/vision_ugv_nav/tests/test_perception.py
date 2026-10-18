import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers')))

from perception.camera_processor import CameraProcessor
from perception.terrain_classifier import TerrainClassifier
from perception.obstacle_detector import ObstacleDetector
from perception.traversability_map import TraversabilityMap

class TestPerception(unittest.TestCase):
    def test_camera_processor_projection(self):
        cam = CameraProcessor(None, None, fov=1.2217)
        depth = np.ones((240, 320), dtype=np.float32) * 3.0
        pts = cam.depth_to_point_cloud(depth, step=8)
        self.assertEqual(pts.shape[0], 3)
        self.assertGreater(pts.shape[1], 100)
        # Forward points should have positive X
        self.assertTrue(np.all(pts[0, :] > 0))

    def test_terrain_classifier(self):
        clf = TerrainClassifier()
        # Create synthetic dirt patch: warm earthy
        dirt_rgb = np.zeros((100, 100, 3), dtype=np.uint8)
        dirt_rgb[:, :] = [160, 130, 80]
        # Create synthetic grass patch: green dominant
        grass_rgb = np.zeros((100, 100, 3), dtype=np.uint8)
        grass_rgb[:, :] = [40, 140, 35]
        
        is_dirt, is_grass, cost = clf.classify_color(dirt_rgb)
        self.assertTrue(np.all(is_dirt))
        self.assertFalse(np.all(is_grass))
        self.assertLess(np.mean(cost), 0.15)
        
        is_dirt_g, is_grass_g, cost_g = clf.classify_color(grass_rgb)
        self.assertTrue(np.all(is_grass_g))
        self.assertGreater(np.mean(cost_g), 0.15)

    def test_obstacle_detector(self):
        detector = ObstacleDetector(min_height=0.08, max_height=1.8, corridor_width=1.0)
        # Point right in corridor: X=2.0, Y=0.0, Z=0.5
        pts = np.array([
            [2.0, 5.0],
            [0.0, 3.5],
            [0.5, 0.4]
        ])
        obs_pts, blocked, min_dist, clusters = detector.detect_obstacles(pts)
        self.assertTrue(blocked)
        self.assertAlmostEqual(min_dist, 2.0, places=1)

    def test_traversability_map_inflation(self):
        tmap = TraversabilityMap(resolution=0.1, size_x=6.0, size_y=6.0, inflation_radius=0.45)
        # Place obstacle at x=2.0, y=0.0
        obs_pts = np.array([[2.0], [0.0]])
        costmap = tmap.update(obs_pts)
        ix, iy = tmap.world_to_grid(2.0, 0.0)
        self.assertEqual(costmap[ix, iy], 100.0)
        
        # Test collision evaluation for a trajectory through the obstacle
        traj_colliding = [(1.5, 0.0), (2.0, 0.0), (2.5, 0.0)]
        is_free, cost, _ = tmap.evaluate_trajectory(traj_colliding)
        self.assertFalse(is_free)
        
        # Trajectory clearly to the side should be free
        traj_free = [(1.5, 1.5), (2.0, 1.5), (2.5, 1.5)]
        is_free_side, cost_side, _ = tmap.evaluate_trajectory(traj_free)
        self.assertTrue(is_free_side)

    def test_yolo_detector(self):
        from perception.yolo_detector import YOLODetector
        yd = YOLODetector(width=320, height=240)
        
        # Test 1: Synthetic Cyan Goal Beacon
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        depth = np.full((240, 320), 4.0, dtype=np.float32)
        # Draw glowing cyan rectangle
        rgb[100:150, 140:180] = [20, 180, 240]
        
        dets, img = yd.detect(rgb, depth, None)
        self.assertTrue(any(d['class'] == 'GOAL_BEACON' for d in dets))
        
        # Test 2: Synthetic Hazard Barrel
        rgb_hazard = np.zeros((240, 320, 3), dtype=np.uint8)
        rgb_hazard[120:170, 130:170] = [240, 60, 20]
        dets_h, img_h = yd.detect(rgb_hazard, depth, None)
        self.assertTrue(any(d['class'] == 'HAZARD_BARREL' for d in dets_h))

    def test_midas_depth_estimator(self):
        from perception.midas_depth_estimator import MiDaSDepthEstimator
        from PIL import Image
        estimator = MiDaSDepthEstimator(width=320, height=240)
        
        # Test synthetic frame
        rgb = np.full((240, 320, 3), 120, dtype=np.uint8)
        # Add high-contrast obstacle block in foreground
        rgb[140:200, 120:200] = 20
        
        disp, est_depth = estimator.estimate_depth(rgb)
        self.assertEqual(disp.shape, (240, 320))
        self.assertEqual(est_depth.shape, (240, 320))
        self.assertTrue(np.all(disp >= 0.0) and np.all(disp <= 1.0))
        
        # Inferno colormap output
        img = estimator.render_midas_colormap(disp)
        self.assertIsInstance(img, Image.Image)
        self.assertEqual(img.size, (320, 240))

    def test_slam_heatmap(self):
        tmap = TraversabilityMap(resolution=0.1, size_x=6.0, size_y=6.0)
        # Update footprint and obstacle
        tmap.update_slam_heatmap(0.0, 0.0, 0.0, obstacles_world=[{'x': 2.0, 'y': 1.0}])
        grid = tmap.get_heatmap_grid(downsample=2)
        self.assertEqual(len(grid), 40)
        self.assertEqual(len(grid[0]), 40)
        # Check non-zero cells exist
        flat = [v for row in grid for v in row]
        self.assertTrue(any(v > 0.3 for v in flat))

if __name__ == '__main__':
    unittest.main()
