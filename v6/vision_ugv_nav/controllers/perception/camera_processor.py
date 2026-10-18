import numpy as np
import math

class CameraProcessor:
    """
    Processes raw camera and depth sensor data from Webots.
    Computes camera intrinsics and 3D point cloud transformation into robot body frame.
    """
    def __init__(self, camera, range_finder, camera_left=None, camera_right=None, camera_rear=None, fov=1.2217, mast_height=0.36, mast_x=0.22, pitch_down_rad=0.21):
        self.camera = camera
        self.range_finder = range_finder
        self.camera_left = camera_left
        self.camera_right = camera_right
        self.camera_rear = camera_rear
        
        self.cameras = {
            "front": camera,
            "left": camera_left,
            "right": camera_right,
            "rear": camera_rear
        }
        
        self.width = camera.getWidth() if camera else 320
        self.height = camera.getHeight() if camera else 240
        self.fov = fov
        self.mast_height = mast_height
        self.mast_x = mast_x
        self.pitch_down = pitch_down_rad
        
        # Camera intrinsics (pinhole model)
        self.fx = (self.width / 2.0) / math.tan(self.fov / 2.0)
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        
        # Rotation matrix from camera optical frame to robot base frame
        cp = math.cos(self.pitch_down)
        sp = math.sin(self.pitch_down)
        self.R_cam_to_robot = np.array([
            [ 0.0, -sp,  cp],  # X_robot = -sp * Y_cam + cp * Z_cam
            [-1.0,  0.0, 0.0],  # Y_robot = -X_cam
            [ 0.0, -cp, -sp]   # Z_robot = -cp * Y_cam - sp * Z_cam
        ], dtype=np.float32)

    def get_rgb_frame(self, cam="front"):
        """Returns RGB frame as (H, W, 3) uint8 numpy array from specified camera."""
        device = self.cameras.get(cam, self.camera) if isinstance(cam, str) else (cam or self.camera)
        if not device:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        raw_img = device.getImage()
        if raw_img is None:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        bgra = np.frombuffer(raw_img, dtype=np.uint8).reshape((self.height, self.width, 4))
        rgb = bgra[:, :, [2, 1, 0]]
        return rgb

    def get_depth_frame(self):
        """Returns Depth frame as (H, W) float32 numpy array in meters."""
        if not self.range_finder:
            return np.full((self.height, self.width), 10.0, dtype=np.float32)
        raw_depth = self.range_finder.getRangeImage()
        if raw_depth is None:
            return np.full((self.height, self.width), 10.0, dtype=np.float32)
        # Webots RangeFinder getRangeImage returns a list of floats or buffer
        depth = np.array(raw_depth, dtype=np.float32).reshape((self.height, self.width))
        # Replace inf/nan with max range
        depth = np.nan_to_num(depth, nan=12.0, posinf=12.0, neginf=0.15)
        return depth

    def depth_to_point_cloud(self, depth, step=4):
        """
        Transforms 2D depth image into 3D points in robot base frame (X_fwd, Y_left, Z_up).
        Subsamples by 'step' for real-time performance.
        """
        v_idx, u_idx = np.mgrid[0:self.height:step, 0:self.width:step]
        z_c = depth[::step, ::step]
        
        valid_mask = (z_c > 0.2) & (z_c < 10.0)
        u_val = u_idx[valid_mask]
        v_val = v_idx[valid_mask]
        z_val = z_c[valid_mask]
        
        x_c = (u_val - self.cx) * z_val / self.fx
        y_c = (v_val - self.cy) * z_val / self.fy
        
        pts_cam = np.stack([x_c, y_c, z_val], axis=0) # (3, N)
        pts_robot = self.R_cam_to_robot @ pts_cam
        pts_robot[0, :] += self.mast_x
        pts_robot[2, :] += self.mast_height
        
        return pts_robot # shape (3, N): row 0=X (fwd), row 1=Y (left), row 2=Z (up)
