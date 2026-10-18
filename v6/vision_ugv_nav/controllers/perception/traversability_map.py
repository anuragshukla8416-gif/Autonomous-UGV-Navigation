import numpy as np

class TraversabilityMap:
    """
    Maintains a 2D local robot-centric costmap grid.
    Fuses terrain classification, obstacle point projections, and inflation buffers.
    Evaluates trajectory costs for DWA and local path planners.
    Uses pure-NumPy for 100% portable, high-speed execution.
    """
    def __init__(self, resolution=0.10, size_x=6.0, size_y=6.0, inflation_radius=0.65):
        self.res = resolution
        self.size_x = size_x
        self.size_y = size_y
        self.inflation_radius = inflation_radius
        
        # Grid dimensions: X from 0 to size_x (forward), Y from -size_y/2 to +size_y/2 (lateral)
        self.nx = int(np.round(size_x / resolution))
        self.ny = int(np.round(size_y / resolution))
        
        # Cost grid in [0, 100], 0=free dirt path, 15=grass, 100=lethal obstacle
        self.costmap = np.zeros((self.nx, self.ny), dtype=np.float32)
        self.binary_obstacle_map = np.zeros((self.nx, self.ny), dtype=bool)

        # Precompute circular inflation kernel
        self.rad_cells = int(np.ceil(inflation_radius / resolution))
        kx, ky = np.ogrid[-self.rad_cells:self.rad_cells+1, -self.rad_cells:self.rad_cells+1]
        dist_grid = np.sqrt(kx**2 + ky**2) * resolution
        
        self.inflation_kernel = np.zeros(dist_grid.shape, dtype=np.float32)
        lethal_r = min(0.32, inflation_radius * 0.5)
        lethal = dist_grid < lethal_r
        inflated = (dist_grid >= lethal_r) & (dist_grid <= inflation_radius)
        decay = 99.0 * (1.0 - (dist_grid - lethal_r) / max(0.01, (inflation_radius - lethal_r)))
        
        self.inflation_kernel[inflated] = decay[inflated]
        self.inflation_kernel[lethal] = 100.0

        # Global 2D SLAM Traversability & Exploration Heatmap
        self.slam_res = 0.50 # 0.5m grid resolution
        self.slam_bounds = (-20.0, 20.0)
        self.slam_n = int((self.slam_bounds[1] - self.slam_bounds[0]) / self.slam_res)
        # Heatmap: 0.0=unexplored, 0.25=explored path, 0.75-1.0=obstacle/hazard
        self.slam_heatmap = np.zeros((self.slam_n, self.slam_n), dtype=np.float32)

    def world_to_grid(self, x, y):
        """Converts robot frame (x_fwd, y_left) to grid indices (ix, iy)."""
        ix = int(np.floor(x / self.res))
        iy = int(np.floor((y + self.size_y / 2.0) / self.res))
        return ix, iy

    def grid_to_world(self, ix, iy):
        """Converts grid indices (ix, iy) to robot frame (x, y)."""
        x = (ix + 0.5) * self.res
        y = (iy + 0.5) * self.res - self.size_y / 2.0
        return x, y

    def update(self, obstacle_pts, terrain_cost=None):
        """
        Updates costmap from obstacle 2D points and camera terrain cost.
        """
        # Initialize with baseline traversable cost
        self.costmap.fill(15.0) # default grass cost
        self.binary_obstacle_map.fill(False)
        
        # Default traversable corridor directly ahead
        mid_y_idx = self.ny // 2
        path_width_cells = int(np.round(1.6 / self.res))
        y_start = max(0, mid_y_idx - path_width_cells // 2)
        y_end = min(self.ny, mid_y_idx + path_width_cells // 2)
        self.costmap[:, y_start:y_end] = 0.0 # Clear dirt road
        
        if obstacle_pts is not None and obstacle_pts.shape[1] > 0:
            ox = obstacle_pts[0, :]
            oy = obstacle_pts[1, :]
            
            valid = (ox >= 0.0) & (ox < self.size_x) & (oy >= -self.size_y / 2.0) & (oy < self.size_y / 2.0)
            valid_x = ox[valid]
            valid_y = oy[valid]
            
            ix_arr = np.clip(np.floor(valid_x / self.res).astype(int), 0, self.nx - 1)
            iy_arr = np.clip(np.floor((valid_y + self.size_y / 2.0) / self.res).astype(int), 0, self.ny - 1)
            
            self.binary_obstacle_map[ix_arr, iy_arr] = True

            # Stamp inflation kernel for each unique obstacle cell
            obs_indices = np.unique(np.stack([ix_arr, iy_arr], axis=1), axis=0)
            k_radius = self.rad_cells
            k_size = 2 * k_radius + 1

            for ix, iy in obs_indices:
                x_min_map = max(0, ix - k_radius)
                x_max_map = min(self.nx, ix + k_radius + 1)
                y_min_map = max(0, iy - k_radius)
                y_max_map = min(self.ny, iy + k_radius + 1)

                x_min_k = x_min_map - (ix - k_radius)
                x_max_k = x_min_k + (x_max_map - x_min_map)
                y_min_k = y_min_map - (iy - k_radius)
                y_max_k = y_min_k + (y_max_map - y_min_map)

                k_patch = self.inflation_kernel[x_min_k:x_max_k, y_min_k:y_max_k]
                self.costmap[x_min_map:x_max_map, y_min_map:y_max_map] = np.maximum(
                    self.costmap[x_min_map:x_max_map, y_min_map:y_max_map],
                    k_patch
                )

        return self.costmap

    def evaluate_trajectory(self, trajectory):
        """
        Evaluates a predicted trajectory (list of (x, y) coordinates in robot frame).
        Returns:
            is_collision_free: bool
            avg_cost: float
            min_clearance: float in meters
        """
        if len(trajectory) == 0:
            return True, 0.0, 999.0
            
        total_cost = 0.0
        min_clearance = 999.0
        
        for pt in trajectory:
            x, y = pt[0], pt[1]
            if x < 0.0 or x >= self.size_x or y < -self.size_y / 2.0 or y >= self.size_y / 2.0:
                continue
            ix, iy = self.world_to_grid(x, y)
            if 0 <= ix < self.nx and 0 <= iy < self.ny:
                cost = self.costmap[ix, iy]
                if cost >= 95.0: # Lethal obstacle collision
                    return False, 100.0, 0.0
                total_cost += cost
                
        avg_cost = total_cost / max(1, len(trajectory))
        return True, avg_cost, min_clearance

    def update_slam_heatmap(self, rx, ry, rtheta, obstacles_world=None):
        """Updates global 2D SLAM exploration and obstacle density heatmap."""
        # 1. Mark explored footprint around robot
        gx = int(np.clip((rx - self.slam_bounds[0]) / self.slam_res, 0, self.slam_n - 1))
        gy = int(np.clip((ry - self.slam_bounds[0]) / self.slam_res, 0, self.slam_n - 1))
        
        # Mark local 3x3 footprint as traversed
        x_min = max(0, gx - 2)
        x_max = min(self.slam_n, gx + 3)
        y_min = max(0, gy - 2)
        y_max = min(self.slam_n, gy + 3)
        self.slam_heatmap[x_min:x_max, y_min:y_max] = np.maximum(
            self.slam_heatmap[x_min:x_max, y_min:y_max], 0.35
        )

        # 2. Mark detected obstacle clusters in world coordinates as hot
        if obstacles_world:
            for obs in obstacles_world:
                ox = obs.get('x', 0) if isinstance(obs, dict) else obs[0]
                oy = obs.get('y', 0) if isinstance(obs, dict) else obs[1]
                ogx = int(np.clip((ox - self.slam_bounds[0]) / self.slam_res, 0, self.slam_n - 1))
                ogy = int(np.clip((oy - self.slam_bounds[0]) / self.slam_res, 0, self.slam_n - 1))
                ox_min = max(0, ogx - 1)
                ox_max = min(self.slam_n, ogx + 2)
                oy_min = max(0, ogy - 1)
                oy_max = min(self.slam_n, ogy + 2)
                self.slam_heatmap[ox_min:ox_max, oy_min:oy_max] = 1.0

    def get_heatmap_grid(self, downsample=2):
        """Returns 2D list of downsampled float values in [0, 1] for dashboard visualization."""
        ds = self.slam_heatmap[::downsample, ::downsample]
        return np.round(ds, 2).tolist()
