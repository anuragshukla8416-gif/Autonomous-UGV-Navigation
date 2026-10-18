import numpy as np
import math

class DWAPlanner:
    """
    Dynamic Window Approach (DWA) local obstacle-avoidance planner.
    Samples velocity pairs (v, w), simulates robot trajectories, and picks
    the trajectory that optimizes target alignment, obstacle clearance, and speed.
    """
    def __init__(self, config=None):
        # Kinematic limits
        self.max_v = 0.85
        self.min_v = -0.15
        self.max_w = 1.20
        self.max_acc_v = 0.80
        self.max_acc_w = 2.0
        
        # Simulation parameters
        self.dt = 0.15
        self.sim_time = 2.2
        self.sim_steps = int(self.sim_time / self.dt)
        self.v_samples = 9
        self.w_samples = 17
        
        # Scoring weights
        self.w_heading = 0.25
        self.w_clearance = 0.40
        self.w_velocity = 0.20
        self.w_traversability = 0.20

    def plan(self, current_v, current_w, robot_pose, lookahead_target, traversability_map):
        """
        Args:
            current_v: float current forward velocity
            current_w: float current angular velocity
            robot_pose: (x, y, theta) in world frame
            lookahead_target: (gx, gy) target waypoint in world frame
            traversability_map: TraversabilityMap instance
        Returns:
            best_v: float
            best_w: float
            best_traj: list of (x, y) in robot frame
            all_trajs: list of (traj, cost) for dashboard visualization
        """
        rx, ry, rtheta = robot_pose
        tx, ty = lookahead_target
        
        # Compute dynamic window [v_min, v_max] x [w_min, w_max]
        v_min = max(self.min_v, current_v - self.max_acc_v * self.dt)
        v_max = min(self.max_v, current_v + self.max_acc_v * self.dt)
        w_min = max(-self.max_w, current_w - self.max_acc_w * self.dt)
        w_max = min(self.max_w, current_w + self.max_acc_w * self.dt)
        
        v_cands = np.linspace(v_min, v_max, self.v_samples)
        w_cands = np.linspace(w_min, w_max, self.w_samples)
        
        best_cost = float('inf')
        best_v = 0.0
        best_w = 0.0
        best_traj = []
        all_trajs = []
        
        # Target angle in robot frame
        dx_target_w = tx - rx
        dy_target_w = ty - ry
        # Rotate into robot frame: (cos*dx + sin*dy, -sin*dx + cos*dy)
        target_x_robot = dx_target_w * math.cos(rtheta) + dy_target_w * math.sin(rtheta)
        target_y_robot = -dx_target_w * math.sin(rtheta) + dy_target_w * math.cos(rtheta)
        
        for v in v_cands:
            for w in w_cands:
                # Simulate trajectory in robot frame starting at (0, 0, 0)
                traj = []
                x, y, th = 0.0, 0.0, 0.0
                for _ in range(self.sim_steps):
                    x += v * math.cos(th) * self.dt
                    y += v * math.sin(th) * self.dt
                    th += w * self.dt
                    traj.append((x, y))
                    
                # Evaluate collision and traversability
                is_free, trav_cost, clearance = traversability_map.evaluate_trajectory(traj)
                if not is_free:
                    continue # Discard colliding trajectories
                    
                # End point alignment to target
                end_x, end_y = traj[-1]
                dist_to_target = math.hypot(target_x_robot - end_x, target_y_robot - end_y)
                
                # Heading alignment: angle between trajectory end heading and vector to target
                angle_to_target = math.atan2(target_y_robot - end_y, target_x_robot - end_x)
                heading_diff = abs(angle_to_target - th)
                while heading_diff > math.pi: heading_diff -= 2 * math.pi
                heading_penalty = abs(heading_diff)
                
                # Cost formulation (lower is better)
                cost = (self.w_heading * heading_penalty +
                        self.w_clearance * (trav_cost / 100.0) +
                        self.w_velocity * ((self.max_v - v) / self.max_v) +
                        self.w_traversability * (dist_to_target * 0.15))
                        
                all_trajs.append((traj, float(cost)))
                
                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w
                    best_traj = traj

        # If no forward trajectory is free (emergency collision threat), stop and turn
        if best_cost == float('inf'):
            best_v = 0.0
            best_w = 0.6 if target_y_robot > 0 else -0.6
            best_traj = [(0.0, 0.0)]

        return float(best_v), float(best_w), best_traj, all_trajs
