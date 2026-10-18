#!/usr/bin/env python3
import os
import sys
import json
import time
from flask import Flask, render_template, jsonify, request, Response

app = Flask(__name__)

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
TELEMETRY_PATH = os.path.join(DATA_DIR, "live_data.json")
SUPERVISOR_PATH = os.path.join(DATA_DIR, "supervisor_data.json")
CMD_PATH = os.path.join(DATA_DIR, "control_cmd.json")
REPORT_PATH = os.path.join(DATA_DIR, "mission_report.json")

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/telemetry")
def get_telemetry():
    telemetry = {}
    supervisor = {}
    report = None

    if os.path.exists(TELEMETRY_PATH):
        try:
            with open(TELEMETRY_PATH, 'r') as f:
                telemetry = json.load(f)
        except Exception:
            pass

    if os.path.exists(SUPERVISOR_PATH):
        try:
            with open(SUPERVISOR_PATH, 'r') as f:
                supervisor = json.load(f)
        except Exception:
            pass

    if os.path.exists(REPORT_PATH):
        try:
            with open(REPORT_PATH, 'r') as f:
                report = json.load(f)
        except Exception:
            pass

    return jsonify({
        "telemetry": telemetry,
        "supervisor": supervisor,
        "report": report
    })

def atomic_write_json(path, data):
    try:
        tmp = path + ".tmp"
        with open(tmp, 'w') as f:
            json.dump(data, f)
        os.replace(tmp, path)
    except Exception as e:
        print(f"[Dashboard Error] Failed atomic write to {path}: {e}")

@app.route("/api/control", methods=["POST"])
def send_control():
    data = request.get_json() or {}
    action = data.get("action", "")
    atomic_write_json(CMD_PATH, {"action": action, "timestamp": time.time()})
    return jsonify({"status": "command_queued", "action": action})

@app.route("/api/set_mission", methods=["POST"])
def set_mission():
    data = request.get_json() or {}
    start_x = data.get("start_x")
    start_y = data.get("start_y")
    goal_x = data.get("goal_x")
    goal_y = data.get("goal_y")
    reset_robot = bool(data.get("reset_robot", False))
    
    cmd_payload = {
        "action": "set_mission",
        "start_x": start_x,
        "start_y": start_y,
        "goal_x": goal_x,
        "goal_y": goal_y,
        "reset_robot": reset_robot,
        "timestamp": time.time()
    }
    atomic_write_json(CMD_PATH, cmd_payload)
    return jsonify({"status": "mission_dispatched", "mission": cmd_payload})

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    print(f"[Dashboard] Launching Judging Dashboard at http://127.0.0.1:{port}")
    app.run(host="0.0.0.0", port=port, debug=False, threaded=True)
