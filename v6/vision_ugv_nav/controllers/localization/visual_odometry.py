import numpy as np
import math

class VisualOdometry:
    """
    Performs visual odometry / optical flow tracking across camera frames.
    Uses pure NumPy/SciPy with optional OpenCV acceleration if installed.
    Estimates frame-to-frame forward displacement (dx) and yaw rotation (dtheta).
    """
    def __init__(self, fx=230.0, fy=230.0, cx=160.0, cy=120.0, max_features=100):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.max_features = max_features
        
        self.prev_gray = None
        self.prev_depth = None
        self.has_cv2 = False
        try:
            import cv2
            self.cv2 = cv2
            self.has_cv2 = True
        except ImportError:
            self.cv2 = None

    def rgb_to_gray(self, rgb):
        """Converts RGB to grayscale float array [0.0, 1.0]."""
        return (0.2989 * rgb[:, :, 0] + 0.5870 * rgb[:, :, 1] + 0.1140 * rgb[:, :, 2]) / 255.0

    def extract_features(self, gray):
        """Selects high-gradient salient corner points in image."""
        h, w = gray.shape
        # Compute image gradients
        gy, gx = np.gradient(gray)
        gradient_mag = gx**2 + gy**2
        
        # Grid-based non-maximum feature extraction
        step = 16
        pts = []
        for r in range(step, h - step, step):
            for c in range(step, w - step, step):
                patch = gradient_mag[r-6:r+6, c-6:c+6]
                if patch.size > 0 and np.max(patch) > 0.005:
                    max_idx = np.unravel_index(np.argmax(patch), patch.shape)
                    pt_r = r - 6 + max_idx[0]
                    pt_c = c - 6 + max_idx[1]
                    pts.append((pt_c, pt_r))
        return pts[:self.max_features]

    def process_frame(self, rgb, depth, dt=0.032):
        """
        Tracks features between consecutive frames and computes (dx_vo, dy_vo, dtheta_vo).
        Returns:
            dx: forward translation (meters)
            dy: lateral translation (meters)
            dtheta: yaw rotation (radians)
            inliers: number of successfully tracked points
        """
        gray = self.rgb_to_gray(rgb)
        
        if self.prev_gray is None or self.prev_depth is None:
            self.prev_gray = gray
            self.prev_depth = depth
            return 0.0, 0.0, 0.0, 0

        h, w = gray.shape
        dx_est = 0.0
        dy_est = 0.0
        dtheta_est = 0.0
        inlier_count = 0

        if self.has_cv2:
            try:
                p0 = self.cv2.goodFeaturesToTrack(
                    (self.prev_gray * 255).astype(np.uint8),
                    maxCorners=self.max_features,
                    qualityLevel=0.03,
                    minDistance=12
                )
                if p0 is not None and len(p0) >= 6:
                    p1, st, err = self.cv2.calcOpticalFlowPyrLK(
                        (self.prev_gray * 255).astype(np.uint8),
                        (gray * 255).astype(np.uint8),
                        p0, None, winSize=(15, 15), maxLevel=2
                    )
                    good_0 = p0[st == 1]
                    good_1 = p1[st == 1]
                    flow = good_1 - good_0
                    
                    # Horizontal flow relates to yaw rotation
                    u_disp = flow[:, 0]
                    # Vertical flow relates to forward translation
                    v_disp = flow[:, 1]
                    
                    # Compute rotation around vertical axis: dtheta ~ -mean(u_disp) / fx
                    dtheta_est = float(-np.median(u_disp) / self.fx)
                    
                    # Compute forward motion from vertical expansion / displacement
                    # Optical flow expansion v_disp: v = fy * Y / Z -> dv = -fy * v_x * Y / Z^2
                    v_med = float(np.median(v_disp))
                    # Median depth of tracked points
                    z_vals = [self.prev_depth[int(np.clip(p[1], 0, h-1)), int(np.clip(p[0], 0, w-1))] for p in good_0]
                    valid_z = [z for z in z_vals if 0.4 < z < 8.0]
                    avg_z = np.median(valid_z) if len(valid_z) > 0 else 2.5
                    dx_est = float(v_med * avg_z / (self.fy * 1.5))
                    inlier_count = len(good_0)
            except Exception:
                inlier_count = 0

        # Robust pure-NumPy patch correlation fallback
        if inlier_count < 6:
            pts = self.extract_features(self.prev_gray)
            displacements_u = []
            displacements_v = []
            patch_r = 5
            search_w = 12
            
            for pt_c, pt_r in pts:
                if pt_r - patch_r < 0 or pt_r + patch_r >= h or pt_c - patch_r < 0 or pt_c + patch_r >= w:
                    continue
                prev_patch = self.prev_gray[pt_r-patch_r:pt_r+patch_r+1, pt_c-patch_r:pt_c+patch_r+1]
                
                # Search window in current frame
                r_min = max(patch_r, pt_r - search_w)
                r_max = min(h - patch_r - 1, pt_r + search_w)
                c_min = max(patch_r, pt_c - search_w)
                c_max = min(w - patch_r - 1, pt_c + search_w)
                
                best_ssd = 999.0
                best_dc, best_dr = 0, 0
                for sr in range(r_min, r_max, 2):
                    for sc in range(c_min, c_max, 2):
                        curr_patch = gray[sr-patch_r:sr+patch_r+1, sc-patch_r:sc+patch_r+1]
                        ssd = np.sum((prev_patch - curr_patch)**2)
                        if ssd < best_ssd:
                            best_ssd = ssd
                            best_dr = sr - pt_r
                            best_dc = sc - pt_c
                            
                if best_ssd < 0.35:
                    displacements_u.append(best_dc)
                    displacements_v.append(best_dr)

            if len(displacements_u) >= 4:
                med_u = float(np.median(displacements_u))
                med_v = float(np.median(displacements_v))
                dtheta_est = float(-med_u / self.fx)
                dx_est = float(med_v * 2.2 / (self.fy * 1.2))
                inlier_count = len(displacements_u)

        self.prev_gray = gray
        self.prev_depth = depth
        return dx_est, dy_est, dtheta_est, inlier_count
