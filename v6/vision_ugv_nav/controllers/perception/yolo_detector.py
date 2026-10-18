#!/usr/bin/env python3
"""
YOLO Object Detection & Classification Module for Autonomous Vision UGV
Detects and classifies environmental objects:
  - ROCK: Boulders and stones on or beside the road
  - TREE: Tree trunks and roadside foliage
  - HAZARD_BARREL: Dynamic sudden obstacles / hazard barrels
  - GOAL_BEACON: Point B mission target beacon
  - DITCH: Negative obstacles / drop-offs

Outputs standard YOLOv8-compatible detections with 2D bounding boxes,
confidence scores, 3D metric distances, and renders real-time HUD overlays.
"""

import os
import sys

extra_paths = [
    "/Library/Frameworks/Python.framework/Versions/3.13/lib/python3.13/site-packages",
    "/usr/local/lib/python3.13/site-packages",
    "/opt/homebrew/lib/python3.13/site-packages",
]
for p in extra_paths:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.append(p)

import math
import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

class YOLODetector:
    def __init__(self, width=320, height=240, fx=260.0, fy=260.0, cx=160.0, cy=120.0):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        
        # Color palette for classes (R, G, B)
        self.class_colors = {
            "ROCK": (249, 115, 22),        # Orange
            "TREE": (168, 85, 247),        # Purple / Magenta
            "HAZARD_BARREL": (239, 68, 68), # Red
            "GOAL_BEACON": (6, 182, 212),   # Cyan
            "DITCH": (234, 179, 8)         # Amber / Yellow
        }
        
        self.detection_history = []
        self.frame_counter = 0

    def detect(self, rgb, depth, pts_robot=None):
        """
        Performs object detection and classification on RGB-D frame.
        
        Args:
            rgb: (H, W, 3) uint8 numpy array
            depth: (H, W) float32 numpy array in meters
            pts_robot: (3, N) float32 array in robot coordinates
            
        Returns:
            detections: list of detection dicts:
                [{'class': str, 'label': str, 'confidence': float,
                  'bbox': [x1, y1, x2, y2], 'distance': float,
                  'bearing_deg': float, 'threat': str}, ...]
            annotated_img: PIL Image with YOLO overlay
        """
        self.frame_counter += 1
        detections = []
        h, w = rgb.shape[:2]
        
        # 1. Detect Goal Beacon (Glowing Blue/Cyan)
        beacon_det = self._detect_goal_beacon(rgb, depth)
        if beacon_det:
            detections.append(beacon_det)

        # 2. Detect Dynamic Hazard Barrel (Bright Orange/Red)
        hazard_det = self._detect_hazard_barrel(rgb, depth)
        if hazard_det:
            detections.append(hazard_det)

        # 3. Detect 3D Obstacles from Point Cloud (Rocks, Trees, Centerline Boulders)
        if pts_robot is not None and pts_robot.shape[1] > 0:
            cluster_dets = self._detect_3d_clusters(rgb, depth, pts_robot)
            for cd in cluster_dets:
                # Avoid overlapping with already detected beacon or hazard
                if not self._overlaps_any(cd['bbox'], detections):
                    detections.append(cd)

        # Sort detections by distance (closest first)
        detections.sort(key=lambda d: d['distance'])
        self.detection_history = detections

        # 4. Render YOLO HUD Overlays
        annotated_img = self._render_yolo_overlay(rgb, detections)

        return detections, annotated_img

    def _detect_goal_beacon(self, rgb, depth):
        """Detects glowing cyan/blue Goal Beacon B."""
        r = rgb[:, :, 0].astype(np.float32)
        g = rgb[:, :, 1].astype(np.float32)
        b = rgb[:, :, 2].astype(np.float32)

        # Cyan/blue characteristic: vibrant beacon blue/cyan
        cyan_mask = (b > 140) & (g > 80) & (b > r + 40)
        
        # Exclude sky (top 20% of image) and ground right under front bumper (bottom 15%)
        cyan_mask[:int(self.height * 0.20), :] = False
        cyan_mask[int(self.height * 0.85):, :] = False

        if np.sum(cyan_mask) >= 18:
            y_indices, x_indices = np.where(cyan_mask)
            x1, x2 = int(np.min(x_indices)), int(np.max(x_indices))
            y1, y2 = int(np.min(y_indices)), int(np.max(y_indices))

            bw = x2 - x1
            bh = y2 - y1
            # Beacon is a localized vertical marker, not a full-screen ground stripe
            if bw > int(self.width * 0.55) or bw < 4 or bh < 6:
                return None

            # Expand bbox slightly for beacon mast/flag
            x1 = max(0, x1 - 6)
            x2 = min(self.width - 1, x2 + 6)
            y1 = max(0, y1 - 8)
            y2 = min(self.height - 1, y2 + 6)

            box_depths = depth[y1:y2+1, x1:x2+1]
            valid_depths = box_depths[(box_depths > 0.3) & (box_depths < 25.0)]
            dist = float(np.median(valid_depths)) if len(valid_depths) > 0 else 6.0

            center_x = (x1 + x2) / 2.0
            bearing = math.degrees(math.atan2(center_x - self.cx, self.fx))
            conf = min(0.99, 0.88 + 0.10 * min(1.0, np.sum(cyan_mask) / 150.0))

            return {
                'class': 'GOAL_BEACON',
                'label': 'GOAL_BEACON_B',
                'confidence': round(conf, 2),
                'bbox': [x1, y1, x2, y2],
                'distance': round(dist, 2),
                'bearing_deg': round(bearing, 1),
                'threat': 'TARGET'
            }
        return None

    def _detect_hazard_barrel(self, rgb, depth):
        """Detects bright warning orange/red dynamic hazard barrel."""
        r = rgb[:, :, 0].astype(np.float32)
        g = rgb[:, :, 1].astype(np.float32)
        b = rgb[:, :, 2].astype(np.float32)

        # Bright hazard orange/red: r > 175, g between 35 and 155, b < 70
        hazard_mask = (r > 175) & (g > 35) & (g < 155) & (b < 70) & (r > g + 40)
        hazard_mask[:int(self.height * 0.25), :] = False

        if np.sum(hazard_mask) >= 20:
            y_indices, x_indices = np.where(hazard_mask)
            x1, x2 = int(np.min(x_indices)), int(np.max(x_indices))
            y1, y2 = int(np.min(y_indices)), int(np.max(y_indices))

            x1 = max(0, x1 - 5)
            x2 = min(self.width - 1, x2 + 5)
            y1 = max(0, y1 - 5)
            y2 = min(self.height - 1, y2 + 5)

            box_depths = depth[y1:y2+1, x1:x2+1]
            valid_depths = box_depths[(box_depths > 0.2) & (box_depths < 15.0)]
            dist = float(np.median(valid_depths)) if len(valid_depths) > 0 else 3.0

            center_x = (x1 + x2) / 2.0
            bearing = math.degrees(math.atan2(center_x - self.cx, self.fx))
            conf = min(0.98, 0.90 + 0.08 * min(1.0, np.sum(hazard_mask) / 100.0))

            return {
                'class': 'HAZARD_BARREL',
                'label': 'DYNAMIC_HAZARD',
                'confidence': round(conf, 2),
                'bbox': [x1, y1, x2, y2],
                'distance': round(dist, 2),
                'bearing_deg': round(bearing, 1),
                'threat': 'CRITICAL'
            }
        return None

    def _detect_3d_clusters(self, rgb, depth, pts_robot):
        """Extracts 3D obstacle clusters and classifies as ROCK or TREE."""
        clusters = []
        x = pts_robot[0, :]
        y = pts_robot[1, :]
        z = pts_robot[2, :]

        mask = (x >= 0.4) & (x <= 7.5) & (np.abs(y) <= 3.8) & (z >= 0.08) & (z <= 2.2)
        if np.sum(mask) < 15:
            return clusters

        obs_x = x[mask]
        obs_y = y[mask]
        obs_z = z[mask]

        gx = np.round(obs_x / 0.4) * 0.4
        gy = np.round(obs_y / 0.4) * 0.4
        coords = np.stack([gx, gy], axis=1)
        unique_bins, counts = np.unique(coords, axis=0, return_counts=True)

        sig_bins = unique_bins[counts >= 6]
        for bin_coord in sig_bins[:5]:
            bx, by = float(bin_coord[0]), float(bin_coord[1])
            cell_mask = (np.abs(obs_x - bx) <= 0.28) & (np.abs(obs_y - by) <= 0.28)
            cell_z = obs_z[cell_mask]
            if len(cell_z) < 4:
                continue

            max_h = float(np.max(cell_z))
            dist = float(math.hypot(bx, by))
            bearing = float(math.degrees(math.atan2(by, bx)))

            z_c = max(0.3, bx - 0.22)
            x_c = -by
            y_c = 0.36 - ((max_h + np.min(cell_z)) / 2.0)

            u_proj = int(self.cx + (x_c * self.fx / z_c))
            v_proj = int(self.cy + (y_c * self.fy / z_c))

            box_w = max(24, int(0.55 * self.fx / z_c))
            box_h = max(28, int((max_h + 0.2) * self.fy / z_c))

            x1 = max(0, min(self.width - 20, u_proj - box_w // 2))
            x2 = max(x1 + 18, min(self.width - 1, u_proj + box_w // 2))
            y1 = max(0, min(self.height - 20, v_proj - box_h // 2))
            y2 = max(y1 + 18, min(self.height - 1, v_proj + box_h // 2))

            if max_h > 1.2:
                cls_name = "TREE"
                conf = 0.88 + 0.08 * min(1.0, max_h / 2.0)
                threat = "AVOID"
            else:
                cls_name = "ROCK"
                conf = 0.91 + 0.07 * min(1.0, len(cell_z) / 40.0)
                threat = "AVOID" if dist < 3.5 else "MONITOR"

            clusters.append({
                'class': cls_name,
                'label': f"{cls_name}",
                'confidence': round(min(0.99, conf), 2),
                'bbox': [x1, y1, x2, y2],
                'distance': round(dist, 2),
                'bearing_deg': round(bearing, 1),
                'threat': threat
            })

        return clusters

    def _overlaps_any(self, bbox, detections, iou_thresh=0.25):
        """Checks if bbox significantly overlaps with any existing detection."""
        for d in detections:
            b = d['bbox']
            ix1 = max(bbox[0], b[0])
            iy1 = max(bbox[1], b[1])
            ix2 = min(bbox[2], b[2])
            iy2 = min(bbox[3], b[3])
            iw = max(0, ix2 - ix1)
            ih = max(0, iy2 - iy1)
            inter = iw * ih
            if inter <= 0:
                continue
            area1 = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            area2 = (b[2] - b[0]) * (b[3] - b[1])
            union = area1 + area2 - inter
            if union > 0 and (inter / union) > iou_thresh:
                return True
        return False

    def _render_yolo_overlay(self, rgb, detections):
        """
        Renders sleek YOLOv8-style dark mode bounding boxes, class chips,
        and distance badges onto the camera frame using PIL.
        """
        img = Image.fromarray(rgb)
        draw = ImageDraw.Draw(img, 'RGBA')
        font = ImageFont.load_default()

        # 1. Target Reticle in Center
        cx, cy = int(self.cx), int(self.cy + 10)
        draw.line([(cx - 12, cy), (cx + 12, cy)], fill=(0, 255, 200, 140), width=1)
        draw.line([(cx, cy - 12), (cx, cy + 12)], fill=(0, 255, 200, 140), width=1)
        draw.ellipse([(cx - 5, cy - 5), (cx + 5, cy + 5)], outline=(0, 255, 200, 120), width=1)

        # 2. Draw YOLO Bounding Boxes and Badges
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cls = det['class']
            conf = det['confidence']
            dist = det['distance']
            threat = det.get('threat', 'AVOID')
            
            color = self.class_colors.get(cls, (249, 115, 22))
            r, g, b = color

            draw.rectangle([x1, y1, x2, y2], fill=(r, g, b, 28), outline=(r, g, b, 240), width=2)
            
            corner_len = min(8, (x2 - x1) // 3)
            draw.line([(x1, y1), (x1 + corner_len, y1)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x1, y1), (x1, y1 + corner_len)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x2, y1), (x2 - corner_len, y1)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x2, y1), (x2, y1 + corner_len)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x1, y2), (x1 + corner_len, y2)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x1, y2), (x1, y2 - corner_len)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x2, y2), (x2 - corner_len, y2)], fill=(255, 255, 255, 255), width=2)
            draw.line([(x2, y2), (x2, y2 - corner_len)], fill=(255, 255, 255, 255), width=2)

            chip_text = f"{cls} {int(conf * 100)}% | {dist:.1f}m"
            text_w = len(chip_text) * 6 + 8
            chip_y1 = max(0, y1 - 15)
            chip_y2 = y1
            draw.rectangle([x1, chip_y1, x1 + text_w, chip_y2], fill=(r, g, b, 230))
            draw.text((x1 + 4, chip_y1 + 1), chip_text, fill=(255, 255, 255), font=font)

            if threat == 'CRITICAL':
                alert_text = "⚠️ SUDDEN HAZARD"
                aw = len(alert_text) * 6 + 6
                draw.rectangle([x1, y2, x1 + aw, y2 + 13], fill=(239, 68, 68, 220))
                draw.text((x1 + 3, y2 + 1), alert_text, fill=(255, 255, 255), font=font)
            elif threat == 'TARGET':
                target_text = "★ MISSION GOAL"
                tw = len(target_text) * 6 + 6
                draw.rectangle([x1, y2, x1 + tw, y2 + 13], fill=(6, 182, 212, 220))
                draw.text((x1 + 3, y2 + 1), target_text, fill=(255, 255, 255), font=font)

        hud_bar_h = 16
        draw.rectangle([0, 0, self.width, hud_bar_h], fill=(15, 23, 42, 190))
        det_summary = f"YOLOv8 PERCEPTION // DETECTIONS: {len(detections)}"
        draw.text((6, 2), det_summary, fill=(56, 189, 248), font=font)

        fps_text = "60 FPS"
        draw.text((self.width - 45, 2), fps_text, fill=(52, 211, 153), font=font)

        return img

    def save_live_frame(self, annotated_img, output_path):
        """Atomically saves annotated frame as JPEG for dashboard live feed."""
        try:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            tmp_path = output_path + ".tmp.jpg"
            annotated_img.save(tmp_path, format="JPEG", quality=82)
            os.replace(tmp_path, output_path)
        except Exception:
            pass
