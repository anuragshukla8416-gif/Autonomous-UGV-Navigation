import numpy as np
import math

class ObstacleDetector:
    """
    Detects 3D obstacles and clusters them in robot Cartesian space.
    Filters out ground plane and tracks hazards directly in the vehicle path.
    """
    def __init__(self, min_height=0.08, max_height=1.8, min_dist=0.3, max_dist=7.5, corridor_width=1.1):
        self.min_height = min_height
        self.max_height = max_height
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.corridor_width = corridor_width
        self.last_closest_dist = 999.0

    def detect_obstacles(self, pts_robot):
        """
        Args:
            pts_robot: (3, N) array in robot base frame (0=X fwd, 1=Y left, 2=Z up).
        Returns:
            obstacle_pts: (2, M) array of (X, Y) obstacle locations on ground plane.
            corridor_blocked: bool, True if an obstacle is directly in vehicle corridor.
            min_corridor_dist: float, distance to nearest obstacle in corridor.
            detected_clusters: list of dicts with cluster info (x, y, radius, dist, bearing).
        """
        if pts_robot is None or pts_robot.shape[1] == 0:
            return np.zeros((2, 0)), False, 999.0, []

        x = pts_robot[0, :]
        y = pts_robot[1, :]
        z = pts_robot[2, :]

        # Filter points by height (above ground, below tree tops) and distance
        valid = (z >= self.min_height) & (z <= self.max_height) &                 (x >= self.min_dist) & (x <= self.max_dist) &                 (np.abs(y) <= 4.0)

        obs_x = x[valid]
        obs_y = y[valid]
        obstacle_pts = np.stack([obs_x, obs_y], axis=0) # (2, M)

        # Check vehicle forward travel corridor
        in_corridor = (np.abs(obs_y) <= (self.corridor_width / 2.0)) & (obs_x >= self.min_dist) & (obs_x <= 4.5)
        corridor_blocked = np.any(in_corridor)
        
        if corridor_blocked:
            min_corridor_dist = float(np.min(obs_x[in_corridor]))
        else:
            min_corridor_dist = 999.0

        self.last_closest_dist = min_corridor_dist

        # Simple spatial binning for cluster detection
        clusters = []
        if len(obs_x) > 10:
            # Discretize into 0.3m grid
            grid_x = np.round(obs_x / 0.3) * 0.3
            grid_y = np.round(obs_y / 0.3) * 0.3
            coords = np.stack([grid_x, grid_y], axis=1)
            unique_coords, counts = np.unique(coords, axis=0, return_counts=True)
            
            # Select cells with significant point density
            sig_cells = unique_coords[counts >= 4]
            for cell in sig_cells[:8]: # cap to 8 prominent obstacles
                cx, cy = float(cell[0]), float(cell[1])
                dist = math.hypot(cx, cy)
                bearing = math.degrees(math.atan2(cy, cx))
                clusters.append({
                    'x': cx,
                    'y': cy,
                    'distance': round(dist, 2),
                    'bearing_deg': round(bearing, 1),
                    'radius': 0.35
                })

        return obstacle_pts, corridor_blocked, min_corridor_dist, clusters
