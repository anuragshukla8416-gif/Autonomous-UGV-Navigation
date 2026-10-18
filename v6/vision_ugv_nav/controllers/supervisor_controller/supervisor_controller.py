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

from controller import Supervisor

def main():
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())
    dt = time_step / 1000.0

    current_dir = os.path.dirname(os.path.abspath(__file__))
    controllers_dir = os.path.dirname(current_dir)
    project_dir = os.path.dirname(controllers_dir)
    data_dir = os.path.join(project_dir, "dashboard")
    os.makedirs(data_dir, exist_ok=True)

    sup_data_path = os.path.join(data_dir, "supervisor_data.json")
    cmd_path = os.path.join(data_dir, "control_cmd.json")
    report_path = os.path.join(data_dir, "mission_report.json")

    ugv_node = supervisor.getFromDef("UGV")
    dyn_obs_node = supervisor.getFromDef("DYNAMIC_OBSTACLE")

    obstacle_nodes = []
    for i in range(25):
        node = supervisor.getFromDef(f"ROCK_{i}")
        if node: obstacle_nodes.append((node, 0.45))
        tnode = supervisor.getFromDef(f"TREE_{i}")
        if tnode: obstacle_nodes.append((tnode, 0.40))
        
    # Centerline boulders
    for c_idx in [1, 2, 3]:
        c_node = supervisor.getFromDef(f"ROCK_CENTER_{c_idx}")
        if c_node: obstacle_nodes.append((c_node, 0.52))

    print(f"[Supervisor] Monitoring UGV and {len(obstacle_nodes)} environmental obstacles.", flush=True)

    gt_trajectory = []
    loc_errors = []
    collision_count = 0
    in_collision_prev = False

    dyn_triggered = False
    goal_reached_confirmed = False

    sim_time = 0.0
    step_count = 0
    mission_finished = False
    last_cmd_ts = time.time()

    try:
        world_fn = supervisor.getWorldPath()
    except Exception:
        world_fn = ""
    goal_x, goal_y = (11.0, 9.0) if "rough" in world_fn else (10.0, 8.0)

    while supervisor.step(time_step) != -1:
        step_count += 1
        sim_time += dt

        if not ugv_node:
            continue

        pos = ugv_node.getPosition()
        rot = ugv_node.getOrientation()
        gt_theta = math.atan2(rot[3], rot[0])
        gt_x, gt_y, gt_z = pos[0], pos[1], pos[2]
        
        gt_trajectory.append((round(gt_x, 3), round(gt_y, 3), round(sim_time, 2)))

        # Check for User/Dashboard Commands
        if os.path.exists(cmd_path):
            try:
                with open(cmd_path, 'r') as f:
                    cmd_data = json.load(f)
                cmd_ts = cmd_data.get("timestamp", 0.0)
                if cmd_ts > last_cmd_ts:
                    last_cmd_ts = cmd_ts
                    action = cmd_data.get("action", "")
                    if action == "inject_obstacle":
                        dyn_triggered = True
                        if dyn_obs_node:
                            # Position dynamic obstacle directly in front of vehicle at 2.8m in ENU frame
                            inj_x = gt_x + 2.8 * math.cos(gt_theta)
                            inj_y = gt_y + 2.8 * math.sin(gt_theta)
                            trans_field = dyn_obs_node.getField("translation")
                            if trans_field:
                                trans_field.setSFVec3f([inj_x, inj_y, 0.45])
                                dyn_obs_node.resetPhysics()
                                # Lateral rolling impulse across UGV path
                                lat_ang = gt_theta + 1.5708
                                dyn_obs_node.setVelocity([0.40 * math.cos(lat_ang), 0.40 * math.sin(lat_ang), 0.0, 0.0, 0.0, 0.0])
                            print(f"[Supervisor] USER INJECTION: Dynamic Hazard Barrel placed at ({inj_x:.2f}, {inj_y:.2f}) directly ahead of UGV!", flush=True)
                    elif action == "set_mission":
                        goal_reached_confirmed = False
                        if "goal_x" in cmd_data and "goal_y" in cmd_data:
                            goal_x = float(cmd_data["goal_x"])
                            goal_y = float(cmd_data["goal_y"])
                            beacon_b = supervisor.getFromDef("BEACON_B")
                            if beacon_b:
                                b_trans = beacon_b.getField("translation")
                                if b_trans:
                                    b_trans.setSFVec3f([goal_x, goal_y, 0.06])
                            print(f"[Supervisor] MISSION DISPATCH: Moved Goal Beacon B to ({goal_x:.2f}, {goal_y:.2f})", flush=True)
                        if "start_x" in cmd_data and "start_y" in cmd_data:
                            start_x = float(cmd_data["start_x"])
                            start_y = float(cmd_data["start_y"])
                            beacon_a = supervisor.getFromDef("BEACON_A")
                            if beacon_a:
                                a_trans = beacon_a.getField("translation")
                                if a_trans:
                                    a_trans.setSFVec3f([start_x, start_y, 0.06])
                            if cmd_data.get("reset_robot") and ugv_node:
                                ugv_trans = ugv_node.getField("translation")
                                ugv_rot = ugv_node.getField("rotation")
                                if ugv_trans:
                                    ugv_trans.setSFVec3f([start_x, start_y, 0.15])
                                if ugv_rot:
                                    target_yaw = math.atan2(goal_y - start_y, goal_x - start_x)
                                    ugv_rot.setSFRotation([0, 0, 1, target_yaw])
                                ugv_node.resetPhysics()
                                print(f"[Supervisor] MISSION DISPATCH: Repositioned UGV to ({start_x:.2f}, {start_y:.2f})", flush=True)
                    elif action == "reset":
                        try:
                            if os.path.exists(cmd_path):
                                os.remove(cmd_path)
                        except Exception:
                            pass
                        supervisor.worldReload()
            except Exception:
                pass

        if dyn_obs_node and not dyn_triggered:
            obs_pos = dyn_obs_node.getPosition()
            dist_to_ugv = math.hypot(obs_pos[0] - gt_x, obs_pos[1] - gt_y)
            if dist_to_ugv < 4.2 and gt_x > -3.5:
                dyn_triggered = True
                inj_x = gt_x + 2.8 * math.cos(gt_theta)
                inj_y = gt_y + 2.8 * math.sin(gt_theta)
                trans_field = dyn_obs_node.getField("translation")
                if trans_field:
                    trans_field.setSFVec3f([inj_x, inj_y, 0.45])
                    dyn_obs_node.resetPhysics()
                    lat_ang = gt_theta + 1.5708
                    dyn_obs_node.setVelocity([0.40 * math.cos(lat_ang), 0.40 * math.sin(lat_ang), 0.0, 0.0, 0.0, 0.0])
                print(f"[Supervisor] AUTO-TRIGGER: Dynamic obstacle launched across UGV path at {dist_to_ugv:.2f}m", flush=True)

        dist_to_goal_gt = math.hypot(goal_x - gt_x, goal_y - gt_y)
        if dist_to_goal_gt <= 1.05 and not goal_reached_confirmed:
            goal_reached_confirmed = True
            print(f"[Supervisor] GROUND TRUTH GOAL REACHED! Distance to Goal B: {dist_to_goal_gt:.2f}m", flush=True)

        est_data = {}
        telemetry_file = os.path.join(data_dir, "live_data.json")
        if os.path.exists(telemetry_file):
            try:
                with open(telemetry_file, 'r') as f:
                    est_data = json.load(f)
            except Exception:
                pass

        est_pose = est_data.get("pose", {})
        est_x = est_pose.get("x", gt_x)
        est_y = est_pose.get("y", gt_y)
        
        loc_err = math.hypot(gt_x - est_x, gt_y - est_y)
        loc_errors.append(loc_err)
        rmse = math.sqrt(sum(e**2 for e in loc_errors) / max(1, len(loc_errors)))

        in_collision_curr = False
        min_clearance = 999.0
        
        for obs_n, obs_r in obstacle_nodes:
            op = obs_n.getPosition()
            d = math.hypot(gt_x - op[0], gt_y - op[1]) - obs_r - 0.26
            if d < min_clearance: min_clearance = d
            if d < 0.0: in_collision_curr = True
                
        if dyn_obs_node:
            d_dyn = math.hypot(gt_x - dyn_obs_node.getPosition()[0], gt_y - dyn_obs_node.getPosition()[1]) - 0.45 - 0.26
            if d_dyn < min_clearance: min_clearance = d_dyn
            if d_dyn < 0.0: in_collision_curr = True

        if in_collision_curr and not in_collision_prev:
            collision_count += 1
            print(f"[Supervisor] COLLISION #{collision_count} DETECTED at pos ({gt_x:.2f}, {gt_y:.2f})", flush=True)
        in_collision_prev = in_collision_curr

        goal_reached = est_data.get("goal_reached", False) or goal_reached_confirmed
        replans = est_data.get("replan_count", 0)
        actual_dist = est_data.get("total_distance", 26.0)
        
        arrival_score = 50.0 if goal_reached else 0.0
        time_score = max(0.0, 20.0 * (1.0 - min(sim_time, 150.0) / 150.0)) if goal_reached else 0.0
        eff_ratio = min(1.0, 26.0 / max(26.0, actual_dist))
        efficiency_score = 15.0 * eff_ratio if goal_reached else 0.0
        collision_score = max(0.0, 10.0 - collision_count * 15.0)
        loc_score = max(0.0, 5.0 * (1.0 - min(rmse, 1.0) / 1.0))
        replan_penalty = min(5.0, replans * 1.0)
        
        total_score = max(0.0, round(arrival_score + time_score + efficiency_score + collision_score + loc_score - replan_penalty, 1))

        if step_count % 4 == 0:
            sup_output = {
                "sim_time": round(sim_time, 2),
                "gt_pose": {"x": round(gt_x, 3), "y": round(gt_y, 3), "theta_deg": round(math.degrees(gt_theta), 1)},
                "gt_trajectory": [(p[0], p[1]) for p in gt_trajectory[-120::3]],
                "loc_error_current": round(loc_err, 3),
                "loc_rmse": round(rmse, 3),
                "collisions": collision_count,
                "min_clearance": round(max(0.0, min_clearance), 2),
                "dyn_obstacle_active": dyn_triggered,
                "score": {
                    "total": total_score,
                    "arrival": arrival_score,
                    "time_bonus": round(time_score, 1),
                    "efficiency": round(efficiency_score, 1),
                    "safety": collision_score,
                    "localization": round(loc_score, 1),
                    "replan_penalty": round(replan_penalty, 1)
                }
            }
            try:
                tmp_path = sup_data_path + ".tmp"
                with open(tmp_path, 'w') as f:
                    json.dump(sup_output, f)
                os.replace(tmp_path, sup_data_path)
            except Exception:
                pass

        if goal_reached and not mission_finished:
            mission_finished = True
            print(f"[Supervisor] MISSION EVALUATION COMPLETE. Final Score: {total_score}/100. RMSE: {rmse:.3f}m, Collisions: {collision_count}", flush=True)

if __name__ == "__main__":
    main()
