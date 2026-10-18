#!/usr/bin/env python3
import subprocess
import time
import os
import sys
import json

def run_integration_test():
    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    world_path = os.path.join(project_dir, 'worlds', 'outdoor_mission_default.wbt')
    data_dir = os.path.join(project_dir, 'dashboard')
    live_path = os.path.join(data_dir, 'live_data.json')
    sup_path = os.path.join(data_dir, 'supervisor_data.json')

    # Remove previous test telemetry
    for p in [live_path, sup_path]:
        if os.path.exists(p):
            try: os.remove(p)
            except Exception: pass

    cmd = [
        '/Applications/Webots.app/Contents/MacOS/webots',
        '--mode=realtime',
        '--batch',
        '--no-rendering',
        '--stdout',
        '--stderr',
        world_path
    ]

    log_path = os.path.join(project_dir, 'tests', 'webots_test_output.log')
    log_file = open(log_path, 'w')

    env = os.environ.copy()
    py313_dir = "/Library/Frameworks/Python.framework/Versions/3.13/bin"
    env["PATH"] = f"{py313_dir}:{env.get('PATH', '')}"
    env["PYTHONPATH"] = f"/Applications/Webots.app/Contents/lib/controller/python:{env.get('PYTHONPATH', '')}"
    env["WEBOTS_HOME"] = "/Applications/Webots.app"

    print(f"[TEST RUNNER] Starting Webots simulation: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, cwd=project_dir, env=env, stdout=log_file, stderr=subprocess.STDOUT, text=True)

    start_time = time.time()
    test_passed = False
    max_test_duration = 90.0 # seconds

    try:
        while (time.time() - start_time) < max_test_duration:
            if proc.poll() is not None:
                print(f"[TEST RUNNER] Webots exited with code {proc.returncode}")
                break

            # Check if live data and supervisor data exist
            if os.path.exists(live_path) and os.path.exists(sup_path):
                try:
                    with open(live_path, 'r') as f:
                        live_data = json.load(f)
                    with open(sup_path, 'r') as f:
                        sup_data = json.load(f)

                    sim_time = live_data.get('time', 0.0)
                    dist_to_goal = live_data.get('dist_to_goal', 99.0)
                    total_dist = live_data.get('total_distance', 0.0)
                    collisions = sup_data.get('collisions', 0)
                    score = sup_data.get('score', {}).get('total', 0.0)
                    rmse = sup_data.get('loc_rmse', 0.0)

                    replans = live_data.get('replan_count', 0)
                    status = live_data.get('status', '')
                    print(f"[SIM PROGRESS] SimTime: {sim_time:.1f}s | DistToGoal: {dist_to_goal:.2f}m | Traveled: {total_dist:.2f}m | Status: {status} | Replans: {replans} | Collisions: {collisions} | RMSE: {rmse:.3f}m | Score: {score}")

                    if (total_dist >= 8.0 and collisions == 0) or live_data.get('goal_reached', False):
                        print(f"[TEST RUNNER] Autonomous navigation verified! Traveled {total_dist:.2f}m without collisions. Replans: {replans}. RMSE: {rmse:.3f}m.", flush=True)
                        test_passed = True
                        break
                except Exception:
                    pass

            time.sleep(1.5)

    finally:
        print("[TEST RUNNER] Terminating Webots simulation...")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        try:
            log_file.close()
        except Exception:
            pass

    if test_passed:
        print("[SUCCESS] Autonomous Outdoor Navigation System Test PASSED!")
        return 0
    else:
        print("[FAILURE] Autonomous navigation test did not meet criteria within time limit.")
        return 1

if __name__ == '__main__':
    sys.exit(run_integration_test())
