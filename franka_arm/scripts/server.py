#!/usr/bin/env python3
"""
Unified Flask server (SYNC):
- /webhook:
    * { "object_retrieve": {...} }  -> run ./object_retrieve_coords.sh x y z qx qy qz qw (wait until done)
    * { "object_retrieve": "grasp" | "release" } -> run ./grasp.sh or ./release.sh (wait until done)
    * { "action": "scene_description", ... } -> capture and return latest image
    * { "action": "object_retrieve" } -> fetch ROS topics (coord_label_base, etc.)
    * { "action": one of SCRIPT_ACTIONS } -> run matching ./<action>.sh (wait until done)
- /health: basic health check
"""

from flask import Flask, request, jsonify, send_file
import subprocess
import os
import time
import glob
import json
import re
import shlex
import datetime

# --- App & Constants ---
app = Flask(__name__)

# Filesystem
SCRIPTS_DIR = os.path.abspath(".")
LOG_DIR = os.path.join(SCRIPTS_DIR, "webhook_logs")
os.makedirs(LOG_DIR, exist_ok=True)

CAPTURE_DIR = "captured_images"

# ROS 2 setup
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
ROS_TOPICS = {
    "coord_label_base": "/coord_label_base",
    "orientation_label": "/orientation_label",
    "end_effector_position": "/end_effector_position",
    "hand_base_position": "/hand_base_position",
}

# Action groups
SCRIPT_ACTIONS = {"greeting", "talk", "yes", "no"}
SCENE_ACTION = "scene_description"
ROS_ACTIONS = {"object_retrieve"}  # legacy fetch-topics style
ALLOWED_OBJECT_ACTIONS = {"grasp", "release"}  # object_retrieve string actions

# --- Helpers: SYNC shell runner with logs (wait until finish) ---
def run_sync_with_shell(command_str, workdir=None, timeout=None):
    """
    Run under bash -lc and BLOCK until completion.
    Returns (returncode, None, log_path, duration_sec) on completion,
            (None, str(error), log_path, duration_sec) on launcher error.
    """
    ts = datetime.datetime.utcnow().strftime("%Y%m%d-%H%M%S")
    log_path = os.path.join(LOG_DIR, f"run-{ts}.log")

    if workdir is None:
        workdir = os.getcwd()
    shell_cmd = f'cd {shlex.quote(workdir)} && {command_str}'

    t0 = time.time()
    try:
        with open(log_path, "ab", buffering=0) as logf:
            completed = subprocess.run(
                ["bash", "-lc", shell_cmd],
                stdout=logf,
                stderr=logf,
                env=os.environ.copy(),
                timeout=timeout,   # set e.g. 180 for a hard cap
                check=False
            )
        duration = round(time.time() - t0, 3)
        return completed.returncode, None, log_path, duration
    except subprocess.TimeoutExpired:
        duration = round(time.time() - t0, 3)
        return 124, f"Command timed out after {duration}s", log_path, duration
    except Exception as e:
        duration = round(time.time() - t0, 3)
        return None, f"Failed to launch: {e}", log_path, duration

def _tail(path, lines=40):
    try:
        with open(path, "rb") as f:
            data = f.read()
        txt = data.decode("utf-8", errors="ignore")
        return "\n".join(txt.splitlines()[-lines:])
    except Exception:
        return None

# --- Helpers for scene_description ---
def run_scene_capture_and_get_latest(topic="/rgb", timeout_sec=5):
    start_time = time.time()
    try:
        result = subprocess.run(
            ["python3", "scene_description.py", "--topic", topic, "--timeout", str(timeout_sec)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False
        )
    except Exception as e:
        return None, f"Failed to launch scene_description.py: {e}"

    if result.returncode != 0:
        return None, f"scene_description.py error: {result.stderr.strip()}"

    pattern = os.path.join(CAPTURE_DIR, "scene_*.jpg")
    files = glob.glob(pattern)
    if not files:
        return None, f"No images found in '{CAPTURE_DIR}'."

    fresh = [p for p in files if os.path.getmtime(p) >= (start_time - 0.5)]
    target_list = fresh if fresh else files
    latest = max(target_list, key=os.path.getmtime) if target_list else None

    if not latest or not os.path.exists(latest):
        return None, "Failed to locate captured image."

    return latest, None

# --- Helpers for ROS 2 topic fetch ---
def run_ros2_echo(topic, timeout_sec=3.0):
    cmd = f'source "{ROS_SETUP}" && ros2 topic echo --once {topic}'
    try:
        out = subprocess.check_output(
            ["bash", "-lc", cmd],
            stderr=subprocess.STDOUT,
            timeout=timeout_sec
        )
        decoded = out.decode("utf-8", errors="ignore").strip()
        if not decoded:
            return None
        return decoded
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        return None

def parse_ros2_output(raw):
    if not raw:
        return None

    # Try std_msgs/String style with quoted JSON in data:
    m_json = re.search(r'data:\s*["\'](.+?)["\']\s*$', raw, flags=re.MULTILINE)
    if m_json:
        s = m_json.group(1)
        try:
            return json.loads(s)
        except Exception:
            return s

    # Try std_msgs/String without quotes, or scalar numbers:
    m_data = re.search(r'^data:\s*(.+)$', raw, flags=re.MULTILINE)
    if m_data:
        val = m_data.group(1).strip()
        if (val.startswith('"') and val.endswith('"')) or (val.startswith("'") and val.endswith("'")):
            val = val[1:-1]
        try:
            return json.loads(val)
        except Exception:
            try:
                return float(val)
            except Exception:
                return val

    # Try geometry-like x/y/z lines:
    coords = {}
    for k in ("x", "y", "z"):
        m = re.search(rf'^\s*{k}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$', raw, flags=re.MULTILINE)
        if m:
            try:
                coords[k] = float(m.group(1))
            except Exception:
                pass
    if len(coords) == 3:
        return coords

    # Generic key: value lines
    lines = [ln for ln in raw.splitlines() if ":" in ln]
    gen = {}
    for ln in lines:
        try:
            k, v = ln.split(":", 1)
            v = v.strip()
            try:
                gen[k.strip()] = float(v)
            except Exception:
                if (v.startswith('"') and v.endswith('"')) or (v.startswith("'") and v.endswith("'")):
                    v = v[1:-1]
                gen[k.strip()] = v
        except Exception:
            pass
    if gen:
        return gen

    return None

def fetch_all_ros_data(timeout_sec=3.0):
    result = {}
    for key, topic in ROS_TOPICS.items():
        raw = run_ros2_echo(topic, timeout_sec=timeout_sec)
        result[key] = parse_ros2_output(raw)
    return result

# --- Routes ---
@app.route("/webhook", methods=["POST"])
def webhook_handler():
    if not request.is_json:
        return jsonify({"status": "error", "message": "Request must be JSON"}), 400

    data = request.get_json(silent=True)
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    # ===== NEW: object_retrieve direct handler (SYNC) =====
    if "object_retrieve" in data:
        action_value = data["object_retrieve"]

        # Case 1: coordinates + quaternion dict
        if isinstance(action_value, dict):
            required_keys = ("x", "y", "z", "qx", "qy", "qz", "qw")
            missing = [k for k in required_keys if k not in action_value]
            if missing:
                return jsonify({
                    "status": "error",
                    "message": f"Coordinates must include {', '.join(required_keys)}"
                }), 400
            try:
                x  = float(action_value["x"])
                y  = float(action_value["y"])
                z  = float(action_value["z"])
                qx = float(action_value["qx"])
                qy = float(action_value["qy"])
                qz = float(action_value["qz"])
                qw = float(action_value["qw"])
            except (TypeError, ValueError):
                return jsonify({"status": "error", "message": "All coordinates and orientation values must be numeric"}), 400

            script = "./object_retrieve_coords.sh"
            # positions to 2 dp; quaternion to 3 dp
            cmd = f'{script} {x:.2f} {y:.2f} {z:.2f} {qx:.3f} {qy:.3f} {qz:.3f} {qw:.3f}'

        # Case 2: string action "grasp"/"release"
        elif isinstance(action_value, str):
            if action_value not in ALLOWED_OBJECT_ACTIONS:
                return jsonify({"status": "error",
                                "message": f"Unknown object_retrieve action '{action_value}'"}), 400
            script = f'./{action_value}.sh'
            cmd = script
        else:
            return jsonify({"status": "error", "message": "'object_retrieve' must be a dict or string"}), 400

        # Ensure script exists
        script_fs = os.path.join(SCRIPTS_DIR, script[2:])  # strip "./"
        if not os.path.isfile(script_fs):
            return jsonify({"status": "error", "message": f"Script '{script}' not found"}), 404

        # Run synchronously (wait until motion/action completes)
        rc, err, log_path, dur = run_sync_with_shell(cmd, workdir=SCRIPTS_DIR, timeout=None)
        log_tail = _tail(log_path, lines=40)

        if err is not None or rc != 0:
            return jsonify({
                "status": "error",
                "message": err or f"Command exited with non-zero return code {rc}",
                "command": cmd,
                "returncode": rc,
                "duration_sec": dur,
                "log": log_path,
                "log_tail": log_tail
            }), 500

        return jsonify({
            "status": "success",
            "command": cmd,
            "returncode": rc,
            "duration_sec": dur,
            "log": log_path,
            "log_tail": log_tail,
            "data_received": data
        }), 200

    # ===== ORIGINAL ACTION FLOW =====
    action = data.get("action")
    if not action:
        return jsonify({"status": "error", "message": "Missing 'action' or 'object_retrieve'"}), 400

    # Scene capture
    if action == SCENE_ACTION:
        topic = data.get("topic", "/rgb")
        timeout = int(data.get("timeout", 5))
        image_path, err = run_scene_capture_and_get_latest(topic=topic, timeout_sec=timeout)
        if err:
            return jsonify({"status": "error", "message": err}), 500
        return send_file(image_path, mimetype="image/jpeg", as_attachment=False)

    # ROS topic retrieval (legacy "action":"object_retrieve")
    if action in ROS_ACTIONS:
        timeout = float(data.get("timeout_sec", 3.0))
        ros_data = fetch_all_ros_data(timeout_sec=timeout)
        return jsonify({"status": "success", "action": action, "topics": ros_data}), 200

    # Script execution (legacy simple scripts) — now SYNC as well
    if action in SCRIPT_ACTIONS:
        script_name = f"{os.path.basename(action)}.sh"
        script_path = os.path.join(".", script_name)
        if not os.path.exists(script_path):
            return jsonify({"status": "error", "message": f"Script '{script_name}' not found"}), 404
        cmd = f'./{script_name}'
        rc, err, log_path, dur = run_sync_with_shell(cmd, workdir=SCRIPTS_DIR, timeout=None)
        log_tail = _tail(log_path, lines=40)
        if err is not None or rc != 0:
            return jsonify({
                "status": "error",
                "message": err or f"Command exited with non-zero return code {rc}",
                "command": cmd,
                "returncode": rc,
                "duration_sec": dur,
                "log": log_path,
                "log_tail": log_tail
            }), 500
        return jsonify({
            "status": "success",
            "command": cmd,
            "returncode": rc,
            "duration_sec": dur,
            "log": log_path,
            "log_tail": log_tail
        }), 200

    return jsonify({"status": "error", "message": f"Unknown action '{action}'"}), 400

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"}), 200

# --- Runner ---
if __name__ == '__main__':
    # Examples:
    # curl -X POST localhost:8000/webhook -H "Content-Type: application/json" \
    # -d '{ "object_retrieve": { "x": 0.40, "y": -0.40, "z": 0.20, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0 } }'
    #
    # curl -X POST localhost:8000/webhook -H "Content-Type: application/json" \
    # -d '{ "object_retrieve": "grasp" }'
    #
    # curl -X POST localhost:8000/webhook -H "Content-Type: application/json" \
    # -d '{ "action": "scene_description", "topic": "/rgb", "timeout": 5 }'
    #
    # curl -X POST localhost:8000/webhook -H "Content-Type: application/json" \
    # -d '{ "action": "object_retrieve", "timeout_sec": 3.0 }'
    #
    # curl -X POST localhost:8000/webhook -H "Content-Type: application/json" \
    # -d '{ "action": "talk" }'
    app.run(host='0.0.0.0', port=8000)

