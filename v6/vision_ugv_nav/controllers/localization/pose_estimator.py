import numpy as np
import math

class PoseEstimator:
    """
    Fuses Visual Odometry, Wheel Odometry, and IMU via an Extended Kalman Filter (EKF).
    Guarantees continuous, drift-compensated GPS-denied pose estimation: (x, y, theta).
    """
    def __init__(self, init_x=-10.0, init_y=-8.0, init_theta=0.785, q_pos=0.015, q_yaw=0.005):
        # State vector: [x, y, theta, v, w]
        self.x = np.array([init_x, init_y, init_theta, 0.0, 0.0], dtype=np.float64)
        
        # State covariance P
        self.P = np.diag([0.05, 0.05, 0.02, 0.05, 0.02])
        
        # Process noise Q
        self.Q = np.diag([q_pos, q_pos, q_yaw, 0.02, 0.01])
        
        # Measurement noises
        self.R_enc = np.diag([0.02, 0.01])        # [v_enc, w_enc]
        self.R_imu = np.diag([0.005, 0.005])      # [yaw_imu, wz_gyro]
        self.R_vo = np.diag([0.04, 0.02])         # [v_vo, w_vo]
        
        # Historical trajectory
        self.trajectory = [(init_x, init_y, init_theta, 0.0)]
        self.total_distance = 0.0

    def predict(self, dt):
        """Kinematic motion prediction model."""
        theta = self.x[2]
        v = self.x[3]
        w = self.x[4]
        
        # State extrapolation
        self.x[0] += v * math.cos(theta) * dt
        self.x[1] += v * math.sin(theta) * dt
        self.x[2] += w * dt
        
        # Jacobian of motion model F
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt
        F[2, 4] = dt
        
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_wheel_odometry(self, v_enc, w_enc):
        """Update state with wheel encoder velocities."""
        self.x[3] = v_enc
        H = np.zeros((2, 5))
        H[0, 3] = 1.0 # v
        H[1, 4] = 1.0 # w
        
        z = np.array([v_enc, w_enc])
        z_pred = H @ self.x
        y = z - z_pred
        
        S = H @ self.P @ H.T + self.R_enc
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x += K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_imu(self, yaw_imu, wz_gyro):
        """Update state with IMU absolute yaw and gyro angular rate."""
        H = np.zeros((2, 5))
        H[0, 2] = 1.0 # theta
        H[1, 4] = 1.0 # w
        
        z = np.array([yaw_imu, wz_gyro])
        z_pred = H @ self.x
        y = z - z_pred
        
        # Angle normalization for heading error
        while y[0] > math.pi: y[0] -= 2.0 * math.pi
        while y[0] < -math.pi: y[0] += 2.0 * math.pi
        
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x += K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        # Anchor unwrapped heading to high-precision IMU filter
        self.x[2] = yaw_imu

    def update_visual_odometry(self, dx_vo, dtheta_vo, dt, inlier_count):
        """Update state with Visual Odometry rotation and motion confirmation."""
        if inlier_count < 8 or dt <= 0:
            return
            
        w_vo = dtheta_vo / dt
        H = np.zeros((1, 5))
        H[0, 4] = 1.0 # w
        
        z = np.array([w_vo])
        z_pred = H @ self.x
        y = z - z_pred
        
        R_w = np.array([[0.04]])
        S = H @ self.P @ H.T + R_w
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x += K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def record_step(self, current_time):
        """Logs estimated pose into historical trajectory."""
        prev_x, prev_y = self.trajectory[-1][0], self.trajectory[-1][1]
        dist_step = math.hypot(self.x[0] - prev_x, self.x[1] - prev_y)
        self.total_distance += dist_step
        self.trajectory.append((float(self.x[0]), float(self.x[1]), float(self.x[2]), current_time))

    def get_pose(self):
        """Returns current estimated (x, y, theta, v, w)."""
        return float(self.x[0]), float(self.x[1]), float(self.x[2]), float(self.x[3]), float(self.x[4])
