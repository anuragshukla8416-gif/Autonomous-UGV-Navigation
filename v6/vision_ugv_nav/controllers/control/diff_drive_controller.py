import numpy as np
import math

class DiffDriveController:
    """
    Converts chassis linear & angular velocities (v, w) to left & right wheel speeds.
    Enforces acceleration limits, wheel saturation, and emergency stop.
    """
    def __init__(self, left_motor, right_motor, track_width=0.46, wheel_radius=0.095, 
                 max_wheel_vel=10.0, max_acc_v=0.8, max_acc_w=2.0):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.track_width = track_width
        self.wheel_radius = wheel_radius
        self.max_wheel_vel = max_wheel_vel
        self.max_acc_v = max_acc_v
        self.max_acc_w = max_acc_w
        
        self.current_v = 0.0
        self.current_w = 0.0
        
        if self.left_motor:
            self.left_motor.setPosition(float('inf'))
            self.left_motor.setVelocity(0.0)
        if self.right_motor:
            self.right_motor.setPosition(float('inf'))
            self.right_motor.setVelocity(0.0)

    def set_cmd_vel(self, target_v, target_w, dt=0.016, emergency_brake=False):
        """
        Enforces acceleration limits and updates motor velocities.
        Returns:
            applied_v: float
            applied_w: float
            left_rad_s: float
            right_rad_s: float
        """
        if emergency_brake:
            self.current_v = 0.0
            self.current_w = 0.0
            if self.left_motor: self.left_motor.setVelocity(0.0)
            if self.right_motor: self.right_motor.setVelocity(0.0)
            return 0.0, 0.0, 0.0, 0.0

        # Acceleration limiting
        dv_max = self.max_acc_v * dt
        dw_max = self.max_acc_w * dt
        
        dv = target_v - self.current_v
        dw = target_w - self.current_w
        
        self.current_v += float(np.clip(dv, -dv_max, dv_max))
        self.current_w += float(np.clip(dw, -dw_max, dw_max))
        
        # Differential drive kinematics:
        # v_l = v - w * (L / 2)
        # v_r = v + w * (L / 2)
        v_l = self.current_v - self.current_w * (self.track_width / 2.0)
        v_r = self.current_v + self.current_w * (self.track_width / 2.0)
        
        # Wheel angular velocities (rad/s)
        w_l = v_l / self.wheel_radius
        w_r = v_r / self.wheel_radius
        
        # Scale down proportionately if exceeding max wheel speed
        max_requested = max(abs(w_l), abs(w_r))
        if max_requested > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_requested
            w_l *= scale
            w_r *= scale
            
        if self.left_motor:
            self.left_motor.setVelocity(float(w_l))
        if self.right_motor:
            self.right_motor.setVelocity(float(w_r))
            
        return self.current_v, self.current_w, float(w_l), float(w_r)

    def stop(self):
        """Brings vehicle to complete stop."""
        self.current_v = 0.0
        self.current_w = 0.0
        if self.left_motor: self.left_motor.setVelocity(0.0)
        if self.right_motor: self.right_motor.setVelocity(0.0)
