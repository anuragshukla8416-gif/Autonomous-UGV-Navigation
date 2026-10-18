#!/usr/bin/env python3
"""
Vision-Based Autonomous Navigation for UGV in Outdoor Environment
1-Click Hackathon Demonstration Launcher
"""

import sys
import os
import time
import signal
import argparse
import subprocess
import webbrowser

def main():
    parser = argparse.ArgumentParser(description="Vision-Based Autonomous UGV Navigation Demo Launcher")
    parser.add_argument("--scenario", type=int, default=1, choices=[1, 2, 3, 4],
                        help="Scenario to run: 1=Normal, 2=Sudden Obstacle, 3=GPS-Denied, 4=Difficult Terrain")
    parser.add_argument("--mode", type=str, default="realtime", choices=["realtime", "fast", "pause"],
                        help="Webots simulation mode (default: realtime)")
    parser.add_argument("--no-gui", action="store_true",
                        help="Run Webots in headless batch mode without 3D window")
    parser.add_argument("--dashboard-only", action="store_true",
                        help="Launch only the web dashboard server")
    parser.add_argument("--port", type=int, default=5005,
                        help="Port for live judging dashboard (default: 5000)")
    args = parser.parse_args()

    project_dir = os.path.dirname(os.path.abspath(__file__))

    # Scenario World Mapping
    world_map = {
        1: "outdoor_mission_default.wbt",
        2: "outdoor_mission_obstacle.wbt",
        3: "outdoor_mission_gps_denied.wbt",
        4: "outdoor_mission_rough.wbt"
    }
    world_file = world_map[args.scenario]
    world_path = os.path.join(project_dir, "worlds", world_file)

    scenario_names = {
        1: "Scenario 1: Normal Outdoor Navigation (A -> B)",
        2: "Scenario 2: Sudden Dynamic Obstacle Injection & Avoidance",
        3: "Scenario 3: Complete GPS-Denied Challenge (Camera + IMU + Odometry)",
        4: "Scenario 4: Difficult Terrain & Narrow Canyon Pass"
    }

    print("=" * 70)
    print("  VISION-BASED AUTONOMOUS UGV NAVIGATION // HACKATHON DEMO")
    print("=" * 70)
    print(f"  Active Scenario : {scenario_names[args.scenario]}")
    print(f"  World File      : {world_file}")
    print(f"  Dashboard URL   : http://127.0.0.1:{args.port}")
    print("=" * 70)

    processes = []

    def cleanup(sig=None, frame=None):
        print("\n[DEMO LAUNCHER] Shutting down simulation and dashboard...")
        for p in processes:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                try: p.kill()
                except Exception: pass
        print("[DEMO LAUNCHER] Shutdown complete.")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Clean stale command and data files from previous runs
    for stale_file in ["control_cmd.json", "live_data.json", "supervisor_data.json"]:
        p = os.path.join(project_dir, "dashboard", stale_file)
        if os.path.exists(p):
            try: os.remove(p)
            except Exception: pass

    # 1. Start Flask Dashboard
    dash_script = os.path.join(project_dir, "dashboard", "app.py")
    env = os.environ.copy()
    env["PORT"] = str(args.port)
    dash_proc = subprocess.Popen([sys.executable, dash_script], cwd=project_dir, env=env)
    processes.append(dash_proc)
    print(f"[1/2] Live Judging Dashboard started on http://127.0.0.1:{args.port}")

    # Give dashboard 1 second to bind
    time.sleep(1.0)
    try:
        webbrowser.open(f"http://127.0.0.1:{args.port}")
    except Exception:
        pass

    if args.dashboard_only:
        print("[DEMO LAUNCHER] Running in dashboard-only mode. Press Ctrl+C to exit.")
        try:
            while True: time.sleep(1)
        except KeyboardInterrupt:
            cleanup()

    # 2. Launch Webots
    webots_bin = "/Applications/Webots.app/Contents/MacOS/webots"
    if not os.path.exists(webots_bin):
        webots_bin = "webots"

    webots_cmd = [
        webots_bin,
        f"--mode={args.mode}",
        "--batch"
    ]
    if args.no_gui:
        webots_cmd.append("--no-rendering")

    webots_cmd.extend(["--stdout", "--stderr", world_path])

    print(f"[2/2] Launching Webots simulation...")
    webots_env = os.environ.copy()
    py313_dir = "/Library/Frameworks/Python.framework/Versions/3.13/bin"
    webots_env["PATH"] = f"{py313_dir}:{webots_env.get('PATH', '')}"
    webots_env["PYTHONPATH"] = f"/Applications/Webots.app/Contents/lib/controller/python:{webots_env.get('PYTHONPATH', '')}"
    webots_env["WEBOTS_HOME"] = "/Applications/Webots.app"

    webots_proc = subprocess.Popen(webots_cmd, cwd=project_dir, env=webots_env)
    processes.append(webots_proc)

    print("\n>>> SIMULATION IS RUNNING! <<<")
    print("Open http://127.0.0.1:5005 in your browser to view the Live Judging Dashboard.")
    print("Press Ctrl+C to stop the demonstration.\n")

    try:
        while True:
            if webots_proc.poll() is not None:
                print(f"[DEMO LAUNCHER] Webots terminated with code {webots_proc.returncode}.")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()

if __name__ == "__main__":
    main()
