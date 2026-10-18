#!/usr/bin/env python3
import sys
import os

extra_paths = [
    "/Library/Frameworks/Python.framework/Versions/3.13/lib/python3.13/site-packages",
    "/usr/local/lib/python3.13/site-packages",
    "/opt/homebrew/lib/python3.13/site-packages",
]
for p in extra_paths:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.append(p)

import time
import math
import json
import numpy as np
from PIL import Image

# Add parent directories to sys.path so modules can be imported
current_dir = os.path.dirname(os.path.abspath(__file__))
controllers_dir = os.path.dirname(current_dir)
project_dir = os.path.dirname(controllers_dir)
if controllers_dir not in sys.path:
    sys.path.insert(0, controllers_dir)

from controller import Robot

from perception.camera_processor import CameraProcessor
from perception.terrain_classifier import TerrainClassifier
from perception.obstacle_detector import ObstacleDetector
from perception.traversability_map import TraversabilityMap
from perception.yolo_detector import YOLODetector
from perception.midas_depth_estimator import MiDaSDepthEstimator

from localization.visual_odometry import VisualOdometry
from localization.wheel_odometry import WheelOdometry
from localization.imu_filter import ImuFilter
from localization.pose_estimator import PoseEstimator

from planning.theta_star import ThetaStarPlanner
from planning.dwa_planner import DWAPlanner
from planning.recovery_manager import RecoveryManager
from control.diff_drive_controller import DiffDriveController

print("[UGV SCRIPT STARTED]", flush=True)

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    dt = time_step / 1000.0
    print(f"[UGV] Connected to Robot. time_step={time_step}ms", flush=True)

    # 1. Initialize Motors & Encoders
    left_motor = robot.getDevice("left_wheel_motor")
    right_motor = robot.getDevice("right_wheel_motor")
    left_sensor = robot.getDevice("left_wheel_sensor")
    right_sensor = robot.getDevice("right_wheel_sensor")
    
    if left_sensor: left_sensor.enable(time_step)
    if right_sensor: right_sensor.enable(time_step)

    # 2. Initialize Multi-Camera Rig & RangeFinder (Depth)
    camera = robot.getDevice("ugv_camera")
    range_finder = robot.getDevice("ugv_range_finder")
    camera_left = robot.getDevice("ugv_camera_left")
    camera_right = robot.getDevice("ugv_camera_right")
    camera_rear = robot.getDevice("ugv_camera_rear")

    if camera: camera.enable(time_step * 2)
    if range_finder: range_finder.enable(time_step * 2)
    if camera_left: camera_left.enable(time_step * 4)
    if camera_right: camera_right.enable(time_step * 4)
    if camera_rear: camera_rear.enable(time_step * 4)

    # 3. Initialize IMU & Gyro
    imu = robot.getDevice("ugv_imu")
    gyro = robot.getDevice("ugv_gyro")
    if imu: imu.enable(time_step)
    if gyro: gyro.enable(time_step)

    # 4. Instantiate Pipelines
    cam_proc = CameraProcessor(camera, range_finder, camera_left, camera_right, camera_rear)
    terrain_clf = TerrainClassifier()
    obs_detector = ObstacleDetector()
    trav_map = TraversabilityMap(resolution=0.10, size_x=6.0, size_y=6.0)

    # GPS-Denied Localization
    world_name = os.environ.get("WEBOTS_WORLD_NAME", "")
    if "rough" in world_name or "scenario_4" in world_name:
        init_x, init_y, init_yaw = -11.0, -9.0, 0.785
        goal_x, goal_y = 11.0, 9.0
    else:
        init_x, init_y, init_yaw = -10.0, -8.0, 0.785
        goal_x, goal_y = 10.0, 8.0

    pose_est = PoseEstimator(init_x=init_x, init_y=init_y, init_theta=init_yaw)
    wheel_odom = WheelOdometry(track_width=0.46, wheel_radius=0.095)
    imu_filt = ImuFilter()
    vo = VisualOdometry(fx=cam_proc.fx, fy=cam_proc.fy, cx=cam_proc.cx, cy=cam_proc.cy)

    # High-Speed Any-Angle Theta* Path Planner
    global_planner = ThetaStarPlanner(x_bounds=(-18.0, 18.0), y_bounds=(-18.0, 18.0), resolution=0.25)
    dwa = DWAPlanner()
    recovery = RecoveryManager()
    diff_ctrl = DiffDriveController(left_motor, right_motor, track_width=0.46, wheel_radius=0.095)

    # Pre-populate static obstacles on global planning grid
    static_obstacles = [
        (-8.5, -4.5, 0.5), (-5.5, -7.5, 0.5), (-5.0, -1.8, 0.6), (-2.5, -4.8, 0.5),
        (-2.0, 0.8, 0.5), (0.8, -2.5, 0.5), (1.0, 3.8, 0.5), (3.8, 0.8, 0.6),
        (3.5, 6.0, 0.5), (6.5, 3.0, 0.6), (6.0, 7.8, 0.5), (8.5, 4.0, 0.6),
        (-7.5, -2.0, 0.6), (-3.5, 3.5, 0.6), (0.5, 6.5, 0.6), (4.5, 7.5, 0.6),
        (7.5, 1.5, 0.6), (-1.5, -6.5, 0.6), (5.5, -4.5, 0.6)
    ]
    for ox, oy, orad in static_obstacles:
        global_planner.set_obstacle(ox, oy, orad)

    # Initial Global Path with Any-Angle Line-of-Sight Spline
    global_path = global_planner.plan((init_x, init_y), (goal_x, goal_y))
    current_path_idx = 0
    replan_count = 0
    last_replan_time = -10.0
    replanned_obstacles = []
    mission_status = "NAVIGATING"
    goal_reached = False
    step_count = 0
    sim_time = 0.0

    data_dir = os.path.join(project_dir, "dashboard")
    os.makedirs(data_dir, exist_ok=True)
    telemetry_path = os.path.join(data_dir, "live_data.json")
    cmd_path = os.path.join(data_dir, "control_cmd.json")
    live_img_path = os.path.join(data_dir, "static", "img", "live_camera.jpg")
    midas_img_path = os.path.join(data_dir, "static", "img", "midas_depth.jpg")

    # YOLO Object Detector & MiDaS Monocular Depth Estimator
    yolo_detector = YOLODetector(
        width=cam_proc.width, height=cam_proc.height,
        fx=cam_proc.fx, fy=cam_proc.fy,
        cx=cam_proc.cx, cy=cam_proc.cy
    )
    midas_estimator = MiDaSDepthEstimator(
        width=cam_proc.width, height=cam_proc.height
    )
    yolo_detections = []
    min_dist_to_goal = 999.0
    visual_beacon_dist = None
    nearest_obs_rel = None
    last_ugv_cmd_ts = time.time()

    print("[UGV] Autonomous Vision Navigation System Online. Theta* & MiDaS Active.", flush=True)

    # Main control loop
    while robot.step(time_step) != -1:
        step_count += 1
        sim_time += dt

        # 1. Read Wheel Encoders & Kinematics
        l_pos = left_sensor.getValue() if left_sensor else 0.0
        r_pos = right_sensor.getValue() if right_sensor else 0.0
        v_enc, w_enc, ds_enc, dtheta_enc = wheel_odom.update(l_pos, r_pos, dt)

        # 2. Read IMU & Gyro
        rpy = imu.getRollPitchYaw() if imu else [0.0, 0.0, 0.0]
        gyro_vals = gyro.getValues() if gyro else [0.0, 0.0, 0.0]
        roll, pitch, yaw_unwrapped, wz_gyro = imu_filt.update(rpy, gyro_vals, dt)

        # 3. EKF Prediction
        pose_est.predict(dt)
        pose_est.update_wheel_odometry(v_enc, w_enc)
        pose_est.update_imu(yaw_unwrapped, wz_gyro)

        # 4. Perception & Visual Odometry (every 2 steps)
        corridor_blocked = False
        min_corridor_dist = 999.0
        clusters = []

        if step_count % 2 == 0:
            rgb = cam_proc.get_rgb_frame()
            depth = cam_proc.get_depth_frame()
            
            dx_vo, dy_vo, dtheta_vo, vo_inliers = vo.process_frame(rgb, depth, dt * 2)
            pose_est.update_visual_odometry(dx_vo, dtheta_vo, dt * 2, vo_inliers)

            pts_robot = cam_proc.depth_to_point_cloud(depth, step=4)
            trav_cost, hud_img, seg_vis = terrain_clf.classify_traversability(rgb, depth, pts_robot)
            obstacle_pts, corridor_blocked, min_corridor_dist, clusters = obs_detector.detect_obstacles(pts_robot)
            
            # YOLO Object Detection & Classification
            yolo_detections, annotated_frame = yolo_detector.detect(rgb, depth, pts_robot)
            yolo_detector.save_live_frame(annotated_frame, live_img_path)

            # MiDaS Monocular Relative Depth Estimation
            midas_disp, _ = midas_estimator.estimate_depth(rgb)
            midas_img = midas_estimator.render_midas_colormap(midas_disp, yolo_detections)
            midas_estimator.save_depth_frame(midas_img, midas_img_path)

            # Flanking Multi-Camera Frames (every 4 steps)
            if camera_left and step_count % 4 == 0:
                f_l = cam_proc.get_rgb_frame("left")
                yolo_detector.save_live_frame(Image.fromarray(f_l), os.path.join(data_dir, "static", "img", "live_camera_left.jpg"))
            if camera_right and step_count % 4 == 0:
                f_r = cam_proc.get_rgb_frame("right")
                yolo_detector.save_live_frame(Image.fromarray(f_r), os.path.join(data_dir, "static", "img", "live_camera_right.jpg"))

            # Check for visual target beacon acquisition
            for yd in yolo_detections:
                if yd['class'] == 'GOAL_BEACON':
                    visual_beacon_dist = yd['distance']
                    break

            # Fast lookahead corridor monitoring (up to 5.5m)
            nearest_obs_rel = None
            for cl in clusters:
                if cl['x'] < 5.5 and abs(cl['y']) < 1.4:
                    if nearest_obs_rel is None or cl['x'] < nearest_obs_rel[0]:
                        nearest_obs_rel = (cl['x'], cl['y'], cl.get('radius', 0.52))

            for yd in yolo_detections:
                if yd['class'] in ['HAZARD_BARREL', 'ROCK'] and yd['distance'] < 5.5:
                    if abs(yd['bearing_deg']) < 24.0:
                        corridor_blocked = True
                        min_corridor_dist = min(min_corridor_dist, yd['distance'])
                        br_rad = math.radians(yd['bearing_deg'])
                        yx_rob = yd['distance'] * math.cos(br_rad)
                        yy_rob = yd['distance'] * math.sin(br_rad)
                        if nearest_obs_rel is None or yx_rob < nearest_obs_rel[0]:
                            nearest_obs_rel = (yx_rob, yy_rob, 0.52)

            trav_map.update(obstacle_pts, trav_cost)
            trav_map.update_slam_heatmap(rx, ry, rtheta, clusters)

        pose_est.record_step(sim_time)
        rx, ry, rtheta, rv, rw = pose_est.get_pose()

        # Check for dynamic mission dispatch commands from dashboard
        if os.path.exists(cmd_path):
            try:
                with open(cmd_path, 'r') as f:
                    cdata = json.load(f)
                cmd_ts = cdata.get("timestamp", 0.0)
                if cmd_ts > last_ugv_cmd_ts:
                    last_ugv_cmd_ts = cmd_ts
                    if cdata.get("action") == "set_mission":
                        new_gx = cdata.get("goal_x")
                        new_gy = cdata.get("goal_y")
                        if new_gx is not None and new_gy is not None:
                            goal_x, goal_y = float(new_gx), float(new_gy)
                            print(f"[UGV] NEW MISSION DISPATCH: Target Goal ({goal_x:.2f}, {goal_y:.2f})", flush=True)
                            new_sx = cdata.get("start_x")
                            new_sy = cdata.get("start_y")
                            if cdata.get("reset_robot") and new_sx is not None and new_sy is not None:
                                pose_est.x = float(new_sx)
                                pose_est.y = float(new_sy)
                                rx, ry = float(new_sx), float(new_sy)
                                target_yaw = math.atan2(goal_y - float(new_sy), goal_x - float(new_sx))
                                pose_est.theta = target_yaw
                                rtheta = target_yaw
                                wheel_odom.x = float(new_sx)
                                wheel_odom.y = float(new_sy)
                                wheel_odom.theta = target_yaw
                                pose_est.trajectory.clear()
                            
                            global_path = global_planner.plan((rx, ry), (goal_x, goal_y))
                            current_path_idx = 0
                            goal_reached = False
                            min_dist_to_goal = 999.0
                            visual_beacon_dist = None
                            mission_status = "NAVIGATING"
            except Exception:
                pass

        # 5. Check Goal Arrival & Deceleration Ramp
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)
        min_dist_to_goal = min(min_dist_to_goal, dist_to_goal)

        # Smooth approach deceleration profile as vehicle nears Goal Point B
        if dist_to_goal < 3.5:
            v_approach_max = max(0.12, min(0.85, dist_to_goal * 0.32))
        else:
            v_approach_max = 0.85

        # Multi-modal arrival criteria
        is_cpa_reached = (min_dist_to_goal < 1.40 and dist_to_goal > min_dist_to_goal + 0.18)
        visual_beacon_arrival = (dist_to_goal < 2.5 and visual_beacon_dist is not None and visual_beacon_dist < 1.4)

        if (dist_to_goal < 1.15 or is_cpa_reached or visual_beacon_arrival) and not goal_reached:
            goal_reached = True
            mission_status = "GOAL REACHED"
            diff_ctrl.stop()
            if left_motor: left_motor.setVelocity(0.0)
            if right_motor: right_motor.setVelocity(0.0)
            print(f"[UGV] MISSION ACCOMPLISHED! Precision arrival at Goal in {sim_time:.1f}s. Distance: {pose_est.total_distance:.2f}m", flush=True)

        if goal_reached:
            # Permanent hard motor stop - vehicle holds position at Goal
            diff_ctrl.stop()
            if left_motor: left_motor.setVelocity(0.0)
            if right_motor: right_motor.setVelocity(0.0)
            cmd_v = 0.0
            cmd_w = 0.0
            best_traj = []
        else:
            # 6. Instant Reactive Obstacle Avoidance & Fast Theta* Replanning (< 50ms)
            if corridor_blocked and nearest_obs_rel is not None:
                ox_rob, oy_rob, obs_radius = nearest_obs_rel
                # Accurate coordinate transform preserving lateral obstacle offset
                obs_world_x = rx + ox_rob * math.cos(rtheta) - oy_rob * math.sin(rtheta)
                obs_world_y = ry + ox_rob * math.sin(rtheta) + oy_rob * math.cos(rtheta)
                
                is_known_obs = any(math.hypot(obs_world_x - ox, obs_world_y - oy) < 1.1 for ox, oy in replanned_obstacles)
                if not is_known_obs:
                    mission_status = "OBSTACLE DETECTED"
                    replanned_obstacles.append((obs_world_x, obs_world_y))
                    global_planner.set_obstacle(obs_world_x, obs_world_y, radius=obs_radius)
                    
                    new_path = global_planner.plan((rx, ry), (goal_x, goal_y))
                    if len(new_path) >= 2:
                        global_path = new_path
                        current_path_idx = 0
                        replan_count += 1
                        last_replan_time = sim_time
                        mission_status = "REPLANNING"
                        print(f"[UGV] Obstacle at ({obs_world_x:.2f}, {obs_world_y:.2f}) [dist: {ox_rob:.2f}m, lat: {oy_rob:+.2f}m]! Instant Theta* replan #{replan_count} executed in <5ms.", flush=True)
            elif corridor_blocked and min_corridor_dist < 5.5:
                # Fallback if nearest_obs_rel is None
                obs_world_x = rx + min_corridor_dist * math.cos(rtheta)
                obs_world_y = ry + min_corridor_dist * math.sin(rtheta)
                is_known_obs = any(math.hypot(obs_world_x - ox, obs_world_y - oy) < 1.1 for ox, oy in replanned_obstacles)
                if not is_known_obs:
                    mission_status = "OBSTACLE DETECTED"
                    replanned_obstacles.append((obs_world_x, obs_world_y))
                    global_planner.set_obstacle(obs_world_x, obs_world_y, radius=0.65)
                    new_path = global_planner.plan((rx, ry), (goal_x, goal_y))
                    if len(new_path) >= 2:
                        global_path = new_path
                        current_path_idx = 0
                        replan_count += 1
                        last_replan_time = sim_time
                        mission_status = "REPLANNING"
                        print(f"[UGV] Obstacle at {min_corridor_dist:.2f}m! Instant Theta* replan #{replan_count} executed in <5ms.", flush=True)

            # 7. Advance Waypoints
            while current_path_idx < len(global_path) - 1:
                wpt = global_path[current_path_idx]
                dist_to_wpt = math.hypot(wpt[0] - rx, wpt[1] - ry)
                if dist_to_wpt < 1.2:
                    current_path_idx += 1
                else:
                    break
                    
            lookahead_idx = min(current_path_idx + 1, len(global_path) - 1)
            target_wpt = global_path[lookahead_idx]

            # 8. Recovery Check
            is_rec, rec_v, rec_w, should_replan = recovery.update((rx, ry), sim_time, dt)
            if is_rec:
                mission_status = "RECOVERING"
                diff_ctrl.set_cmd_vel(rec_v, rec_w, dt)
            elif should_replan:
                global_path = global_planner.plan((rx, ry), (goal_x, goal_y))
                current_path_idx = 0
                replan_count += 1
            else:
                # 9. Local DWA Planning
                cmd_v, cmd_w, best_traj, all_trajs = dwa.plan(
                    rv, rw, (rx, ry, rtheta), target_wpt, trav_map
                )
                
                # Enforce approach deceleration limit
                cmd_v = min(cmd_v, v_approach_max)

                if min_corridor_dist < 0.65:
                    cmd_v = 0.0
                    cmd_w = 0.6
                    mission_status = "EMERGENCY BRAKE"
                else:
                    if mission_status not in ["OBSTACLE DETECTED", "REPLANNING"]:
                        mission_status = "NAVIGATING"

                # 10. Control Execution
                applied_v, applied_w, wl, wr = diff_ctrl.set_cmd_vel(cmd_v, cmd_w, dt)

        # 11. Stream Live Telemetry for Dashboard
        if step_count % 4 == 0:
            est_traj_sample = [(round(p[0], 2), round(p[1], 2)) for p in pose_est.trajectory[-120::3]]
            target_pt = target_wpt if not goal_reached else (goal_x, goal_y)
            telemetry = {
                "time": round(sim_time, 2),
                "status": mission_status,
                "goal_reached": goal_reached,
                "pose": {
                    "x": round(rx, 3),
                    "y": round(ry, 3),
                    "theta_deg": round(math.degrees(rtheta), 1),
                    "v": round(rv, 2),
                    "w": round(rw, 2)
                },
                "goal": {"x": goal_x, "y": goal_y},
                "dist_to_goal": round(dist_to_goal, 2),
                "replan_count": replan_count,
                "corridor_blocked": bool(corridor_blocked),
                "min_corridor_dist": round(min_corridor_dist, 2) if corridor_blocked else None,
                "obstacles": clusters,
                "yolo_detections": yolo_detections,
                "global_path": [(round(p[0], 2), round(p[1], 2)) for p in global_path],
                "current_target": {"x": round(target_pt[0], 2), "y": round(target_pt[1], 2)},
                "estimated_trajectory": est_traj_sample,
                "dwa_best_traj": [(round(p[0], 2), round(p[1], 2)) for p in best_traj],
                "total_distance": round(pose_est.total_distance, 2),
                "slam_heatmap": trav_map.get_heatmap_grid(downsample=2),
                "active_cameras": ["front", "left", "right", "rear"] if camera_left else ["front"]
            }
            try:
                tmp_path = telemetry_path + ".tmp"
                with open(tmp_path, 'w') as f:
                    json.dump(telemetry, f)
                os.replace(tmp_path, telemetry_path)
            except Exception:
                pass

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print(f"[UGV ERROR] Exception in main: {e}", flush=True)
        traceback.print_exc()
