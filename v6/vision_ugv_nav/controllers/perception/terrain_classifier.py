import numpy as np

class TerrainClassifier:
    """
    Classifies outdoor terrain into traversable dirt road, open grass,
    rough terrain, and obstacles using classical vision & geometric surface normal analysis.
    Modular structure allows swapping with deep segmentation models.
    """
    def __init__(self, color_weight=0.35, slope_weight=0.65, traversability_threshold=0.48):
        self.color_weight = color_weight
        self.slope_weight = slope_weight
        self.traversability_threshold = traversability_threshold

    def classify_color(self, rgb):
        """
        Analyzes RGB image to detect dirt path vs grass vs obstacles.
        Returns:
            dirt_mask: boolean mask (H, W)
            grass_mask: boolean mask (H, W)
            color_cost: float array (H, W) in [0.0, 1.0] (0=smooth path, 0.3=grass, 1.0=obstacle)
        """
        r = rgb[:, :, 0].astype(np.float32)
        g = rgb[:, :, 1].astype(np.float32)
        b = rgb[:, :, 2].astype(np.float32)
        
        # Dirt path characteristics: warm earthy brownish-tan, r > b, g > b, moderate saturation
        # r is highest or close to g, b is lowest
        is_dirt = (r > 100) & (g > 80) & (b > 50) & (r > b + 15) & (g > b + 5) & (np.abs(r - g) < 45)
        
        # Grass characteristics: green dominant, g > r and g > b
        is_grass = (g > r + 10) & (g > b + 15) & (g > 60)
        
        # Obstacles / rocks: dark gray, high contrast, or saturated hazard colors
        color_cost = np.ones(rgb.shape[:2], dtype=np.float32) * 0.8
        color_cost[is_grass] = 0.25
        color_cost[is_dirt] = 0.05
        
        return is_dirt, is_grass, color_cost

    def classify_traversability(self, rgb, depth, pts_robot=None):
        """
        Fuses color classification with geometric height and slope analysis.
        Returns:
            traversability_cost: (H, W) in [0.0, 1.0]
            segmentation_mask: (H, W, 3) colored visualization:
                - GREEN: Traversable dirt path
                - YELLOW: Open grass / rough ground
                - RED: Non-traversable obstacle / steep hazard
        """
        h, w = rgb.shape[:2]
        is_dirt, is_grass, color_cost = self.classify_color(rgb)
        
        # Geometry-based cost from depth gradient
        # Compute local depth differences along vertical and horizontal axes
        depth_safe = np.clip(depth, 0.2, 10.0)
        grad_y = np.abs(np.diff(depth_safe, axis=0, append=depth_safe[-1:, :]))
        grad_x = np.abs(np.diff(depth_safe, axis=1, append=depth_safe[:, -1:]))
        slope_metric = np.sqrt(grad_y**2 + grad_x**2)
        
        # Steep jumps (> 0.25m per pixel step) indicate vertical obstacles or ditch drop-offs
        geom_cost = np.clip(slope_metric / 0.35, 0.0, 1.0)
        
        # Bottom portion of the camera image is nearby ground; smooth it
        ground_bias = np.linspace(0.8, 0.1, h)[:, None]
        geom_cost *= ground_bias
        
        fused_cost = self.color_weight * color_cost + self.slope_weight * geom_cost
        fused_cost = np.clip(fused_cost, 0.0, 1.0)
        
        # Generate visually distinct segmentation overlay for hackathon judges
        seg_vis = np.zeros((h, w, 3), dtype=np.uint8)
        # Red for obstacles (cost >= threshold)
        obs_mask = fused_cost >= self.traversability_threshold
        # Green for clear dirt road
        path_mask = (fused_cost < 0.25) & is_dirt
        # Yellow for traversable grass / secondary
        grass_or_alt = (~obs_mask) & (~path_mask)
        
        seg_vis[obs_mask] = [230, 40, 40]       # Red
        seg_vis[grass_or_alt] = [220, 200, 30]  # Yellow
        seg_vis[path_mask] = [40, 220, 80]      # Bright Green
        
        # Blend with original RGB for HUD display
        hud_display = (0.55 * rgb + 0.45 * seg_vis).astype(np.uint8)
        
        return fused_cost, hud_display, seg_vis
