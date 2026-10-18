import heapq
import math
import numpy as np

class AStarPlanner:
    """
    Global A* path planner on a 2D traversability grid.
    Finds safest, optimal path from Point A to Point B avoiding rocks, trees, and steep ditches.
    """
    def __init__(self, x_bounds=(-18.0, 18.0), y_bounds=(-18.0, 18.0), resolution=0.25):
        self.x_min, self.x_max = x_bounds
        self.y_min, self.y_max = y_bounds
        self.res = resolution
        
        self.nx = int(np.round((self.x_max - self.x_min) / resolution))
        self.ny = int(np.round((self.y_max - self.y_min) / resolution))
        
        # Static global cost grid
        self.grid = np.zeros((self.nx, self.ny), dtype=np.float32)

    def world_to_grid(self, x, y):
        ix = int(np.clip(np.floor((x - self.x_min) / self.res), 0, self.nx - 1))
        iy = int(np.clip(np.floor((y - self.y_min) / self.res), 0, self.ny - 1))
        return ix, iy

    def grid_to_world(self, ix, iy):
        x = self.x_min + (ix + 0.5) * self.res
        y = self.y_min + (iy + 0.5) * self.res
        return x, y

    def set_obstacle(self, x, y, radius, cost=100.0):
        """Marks an obstacle circle on the global planning grid with safety padding."""
        pad_r = radius + 0.35 # safety margin
        ix_min, iy_min = self.world_to_grid(x - pad_r, y - pad_r)
        ix_max, iy_max = self.world_to_grid(x + pad_r, y + pad_r)
        
        for ix in range(ix_min, ix_max + 1):
            for iy in range(iy_min, iy_max + 1):
                wx, wy = self.grid_to_world(ix, iy)
                dist = math.hypot(wx - x, wy - y)
                if dist <= radius:
                    self.grid[ix, iy] = max(self.grid[ix, iy], cost)
                elif dist <= pad_r:
                    # Soft buffer
                    decay = cost * (1.0 - (dist - radius) / 0.35)
                    self.grid[ix, iy] = max(self.grid[ix, iy], decay)

    def plan(self, start_pos, goal_pos):
        """
        Runs A* search from start_pos (x, y) to goal_pos (x, y).
        Returns list of (x, y) waypoints.
        """
        sx, sy = start_pos[0], start_pos[1]
        gx, gy = goal_pos[0], goal_pos[1]
        
        start_idx = self.world_to_grid(sx, sy)
        goal_idx = self.world_to_grid(gx, gy)
        
        if start_idx == goal_idx:
            return [(sx, sy), (gx, gy)]

        # 8-connected neighbors: (dx, dy, step_cost)
        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]
        
        # Priority queue: (f_score, h_score, current_node)
        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, start_idx))
        
        came_from = {}
        g_score = {start_idx: 0.0}
        
        def heuristic(idx1, idx2):
            dx = (idx1[0] - idx2[0]) * self.res
            dy = (idx1[1] - idx2[1]) * self.res
            return math.hypot(dx, dy)

        visited = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal_idx:
                # Reconstruct path
                path_idx = [current]
                while current in came_from:
                    current = came_from[current]
                    path_idx.append(current)
                path_idx.reverse()
                
                # Convert back to world coordinates with smoothing
                raw_path = [self.grid_to_world(ix, iy) for (ix, iy) in path_idx]
                raw_path[0] = (sx, sy)
                raw_path[-1] = (gx, gy)
                return self.smooth_path(raw_path)
                
            curr_g = g_score[current]
            
            for dx, dy, step_len in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                
                if not (0 <= nx < self.nx and 0 <= ny < self.ny):
                    continue
                    
                cell_cost = self.grid[nx, ny]
                if cell_cost >= 90.0: # Impassable
                    continue
                    
                tentative_g = curr_g + (step_len * self.res) * (1.0 + cell_cost * 0.05)
                nbr = (nx, ny)
                
                if nbr not in g_score or tentative_g < g_score[nbr]:
                    came_from[nbr] = current
                    g_score[nbr] = tentative_g
                    h = heuristic(nbr, goal_idx)
                    f = tentative_g + h
                    heapq.heappush(open_set, (f, h, nbr))

        # Fallback straight line if path blocked
        return [(sx, sy), (gx, gy)]

    def smooth_path(self, path):
        """Subsamples path points to smooth waypoints with ~1.0m spacing."""
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        for pt in path[1:-1]:
            last_pt = smoothed[-1]
            if math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1]) >= 0.8:
                smoothed.append(pt)
        smoothed.append(path[-1])
        return smoothed
