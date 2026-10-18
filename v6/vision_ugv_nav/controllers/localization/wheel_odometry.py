import math

class WheelOdometry:
    """
    Tracks differential drive kinematics from wheel position sensors.
    """
    def __init__(self, track_width=0.46, wheel_radius=0.095):
        self.track_width = track_width
        self.wheel_radius = wheel_radius
        self.prev_left_pos = None
        self.prev_right_pos = None

    def update(self, left_pos, right_pos, dt):
        """
        Returns:
            v: linear velocity (m/s)
            w: angular velocity (rad/s)
            ds: forward displacement (m)
            dtheta: heading change (rad)
        """
        if self.prev_left_pos is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            return 0.0, 0.0, 0.0, 0.0

        d_phi_l = left_pos - self.prev_left_pos
        d_phi_r = right_pos - self.prev_right_pos
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

        ds_l = self.wheel_radius * d_phi_l
        ds_r = self.wheel_radius * d_phi_r

        ds = (ds_r + ds_l) / 2.0
        dtheta = (ds_r - ds_l) / self.track_width

        dt_safe = max(dt, 0.001)
        v = ds / dt_safe
        w = dtheta / dt_safe

        return v, w, ds, dtheta
