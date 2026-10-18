import time
import math

class RecoveryManager:
    """
    Detects if the vehicle is stuck (no forward progress or oscillating against obstacles)
    and executes deterministic recovery behaviors:
    1. Emergency Stop + Replan
    2. Controlled Reverse Maneuver
    3. Rotate-to-clear field of view
    """
    def __init__(self, stuck_time_threshold=3.0, progress_dist_threshold=0.25, check_interval=1.0):
        self.stuck_time_thresh = stuck_time_threshold
        self.progress_dist_thresh = progress_dist_threshold
        self.check_interval = check_interval
        
        self.last_check_time = 0.0
        self.last_check_pos = None
        self.stuck_start_time = None
        self.is_recovering = False
        self.recovery_stage = 0 # 0: normal, 1: reverse, 2: rotate
        self.recovery_timer = 0.0

    def update(self, current_pos, current_time, dt):
        """
        Returns:
            is_recovering: bool
            rec_v: float linear velocity command
            rec_w: float angular velocity command
            should_replan: bool
        """
        rx, ry = current_pos[0], current_pos[1]
        
        if self.is_recovering:
            self.recovery_timer += dt
            if self.recovery_stage == 1:
                # Stage 1: Back up smoothly for 1.6s
                if self.recovery_timer < 1.6:
                    return True, -0.22, 0.0, False
                else:
                    self.recovery_stage = 2
                    self.recovery_timer = 0.0
            elif self.recovery_stage == 2:
                # Stage 2: Rotate in place to re-scan for 1.8s
                if self.recovery_timer < 1.8:
                    return True, 0.0, 0.65, False
                else:
                    # Recovery complete, trigger replan
                    self.is_recovering = False
                    self.recovery_stage = 0
                    self.stuck_start_time = None
                    self.last_check_time = current_time
                    self.last_check_pos = (rx, ry)
                    return False, 0.0, 0.0, True

        # Check for stuck condition
        if self.last_check_pos is None:
            self.last_check_pos = (rx, ry)
            self.last_check_time = current_time
            return False, 0.0, 0.0, False

        time_elapsed = current_time - self.last_check_time
        if time_elapsed >= self.check_interval:
            dist_moved = math.hypot(rx - self.last_check_pos[0], ry - self.last_check_pos[1])
            if dist_moved < self.progress_dist_thresh:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                elif (current_time - self.stuck_start_time) >= self.stuck_time_thresh:
                    # Vehicle is stuck! Initiate recovery
                    self.is_recovering = True
                    self.recovery_stage = 1
                    self.recovery_timer = 0.0
                    return True, -0.22, 0.0, True
            else:
                self.stuck_start_time = None

            self.last_check_pos = (rx, ry)
            self.last_check_time = current_time

        return False, 0.0, 0.0, False
