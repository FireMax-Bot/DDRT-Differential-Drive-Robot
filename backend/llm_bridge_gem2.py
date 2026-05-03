"""
DDRT — Python Flask LLM Bridge
Improvements:
  - Odometry only updated when ESP32 confirms 200 OK
  - Input validation on /command
  - Inter-step settle delay after each physical move
  - Stricter Gemini prompt with value range constraints
  - API key loaded from environment variable (falls back to inline for dev)
  - /status endpoint for dashboard health check
  - Clean logging throughout
"""

import os
import re
import json
import time
import math
import requests
from flask import Flask, request, jsonify
from flask_cors import CORS
from google import genai

# ===== Configuration =====
ESP_IP      = "ESP_IP"
GEMINI_KEY = os.environ.get("GEMINI_API_KEY")
# NOTE: for production, set GEMINI_API_KEY in your environment and
# remove the fallback string above so it doesn't leak into version control.

INTER_STEP_DELAY = 0.2   # seconds to pause between LLM steps (robot settle time)
ESP_TIMEOUT      = 25    # seconds to wait for ESP32 to finish each move

client = genai.Client(api_key=GEMINI_KEY)
app    = Flask(__name__)
CORS(app)

# ===== Python-side odometry =====
robot_x     = 0.0
robot_y     = 0.0
robot_theta = 0.0
path_history = [{"x": 0, "y": 0}]


# ============================================================
#  HELPERS
# ============================================================

def extract_json(text: str) -> str | None:
    """Pull the first [...] JSON array out of a string."""
    match = re.search(r"\[.*\]", text, re.DOTALL)
    return match.group(0) if match else None


def get_motion_plan(user_text: str) -> str:
    """
    Send natural language to Gemini and get back a JSON motion plan.
    Retries 3 times on failure. Returns a safe fallback on total failure.
    """
    prompt = f"""
You control a differential drive robot.
Convert the instruction below into a compact JSON motion plan.

Rules:
  - "forward" or "backward" -> value is metres, range 0.01 to 2.0
  - "left" or "right"       -> value is degrees, range 1 to 360
  - Use the minimum number of steps needed
  - Output ONLY a valid JSON array — no explanation, no markdown fences

Output format (exactly):
[
  {{"action":"forward","value":0.5}},
  {{"action":"right","value":90}}
]

Instruction: {user_text}
"""

    for attempt in range(1, 4):
        try:
            response = client.models.generate_content(
                model="gemini-2.5-flash",
                contents=prompt
            )
            raw_text  = response.text.strip()
            json_text = extract_json(raw_text)

            if json_text is None:
                raise ValueError("No JSON array found in Gemini response")

            # Validate it parses correctly before returning
            parsed = json.loads(json_text)

            # Clamp values to safe physical limits
            for step in parsed:
                action = step.get("action", "")
                value  = float(step.get("value", 0))
                if action in ("forward", "backward"):
                    step["value"] = max(0.01, min(value, 2.0))
                elif action in ("left", "right"):
                    step["value"] = max(1.0, min(value, 360.0))

            clamped = json.dumps(parsed)
            print(f"[Gemini] Attempt {attempt} OK: {clamped}")
            return clamped

        except Exception as e:
            print(f"[Gemini] Attempt {attempt} failed: {e}")
            if attempt < 3:
                time.sleep(2)

    print("[Gemini] All retries failed — using safe fallback (forward 0.1 m)")
    return json.dumps([{"action": "forward", "value": 0.1}])


def execute_plan(plan_text: str):
    """
    Walk through the JSON plan step by step.
    Sends synchronous HTTP requests to the ESP32 and waits for each
    physical move to complete before continuing.

    FIX: odometry is only updated if ESP32 returns HTTP 200.
         On network error, odometry is NOT updated (robot didn't move).
    """
    global robot_x, robot_y, robot_theta, path_history

    try:
        steps = json.loads(plan_text)
    except Exception as e:
        print(f"[Plan] Failed to parse JSON: {e}")
        return

    for i, step in enumerate(steps):
        action = step["action"]
        value  = float(step["value"])

        print(f"[Step {i+1}/{len(steps)}] {action}  {value}")

        success = False

        if action in ("forward", "backward"):
            distance_cm = value * 100.0
            url = f"http://{ESP_IP}/moveExact?action={action}&distance={distance_cm}"
            try:
                response = requests.get(url, timeout=ESP_TIMEOUT)
                if response.status_code == 200:
                    success = True
                    print(f"  ESP32 OK: {response.text}")
                else:
                    print(f"  ESP32 rejected (status {response.status_code}) — skipping odometry")
            except Exception as e:
                print(f"  ESP32 network error: {e} — skipping odometry")

            # FIX: only update Python odometry if the move actually happened
            if success:
                if action == "forward":
                    robot_x += value * math.cos(robot_theta)
                    robot_y += value * math.sin(robot_theta)
                else:
                    robot_x -= value * math.cos(robot_theta)
                    robot_y -= value * math.sin(robot_theta)
                path_history.append({"x": round(robot_x, 4), "y": round(robot_y, 4)})

        elif action in ("left", "right"):
            url = f"http://{ESP_IP}/turnExact?direction={action}&angle={value}"
            try:
                response = requests.get(url, timeout=ESP_TIMEOUT)
                if response.status_code == 200:
                    success = True
                    print(f"  ESP32 OK: {response.text}")
                else:
                    print(f"  ESP32 rejected (status {response.status_code}) — skipping odometry")
            except Exception as e:
                print(f"  ESP32 network error: {e} — skipping odometry")

            if success:
                radians = math.radians(value)
                if action == "left":
                    robot_theta += radians
                else:
                    robot_theta -= radians
        else:
            print(f"  Unknown action '{action}' — skipped")
            continue

        # Give the robot time to mechanically settle before the next command
        if success and i < len(steps) - 1:
            time.sleep(INTER_STEP_DELAY)


# ============================================================
#  FLASK ROUTES
# ============================================================

@app.route("/command", methods=["POST"])
def command():
    """Receive a natural language command, plan it, execute it."""
    data = request.get_json(silent=True)

    # FIX: input validation
    if not data or "text" not in data or not data["text"].strip():
        return jsonify({"error": "Missing 'text' field in request body"}), 400

    user_text = data["text"].strip()
    print(f"\n[CMD] User: '{user_text}'")

    plan = get_motion_plan(user_text)
    print(f"[CMD] Plan: {plan}")

    execute_plan(plan)

    return jsonify({
        "status": "executed",
        "command": user_text,
        "plan": json.loads(plan),
        "final_pos": {"x": robot_x, "y": robot_y, "theta": robot_theta}
    })


@app.route("/path")
def path():
    """Return the path history for the canvas map."""
    return jsonify(path_history)


@app.route("/reset", methods=["GET", "POST"])
def reset():
    """Reset Python-side odometry and path history."""
    global robot_x, robot_y, robot_theta, path_history
    robot_x      = 0.0
    robot_y      = 0.0
    robot_theta  = 0.0
    path_history = [{"x": 0, "y": 0}]
    print("[RESET] Python odometry cleared.")
    return jsonify({"status": "reset"})


@app.route("/status")
def status():
    """Health check — also tries to ping ESP32 for live connection status."""
    esp_reachable = False
    try:
        r = requests.get(f"http://{ESP_IP}/position", timeout=2)
        esp_reachable = (r.status_code == 200)
        esp_data = r.json() if esp_reachable else {}
    except Exception:
        esp_data = {}

    return jsonify({
        "python": "ok",
        "esp32_reachable": esp_reachable,
        "esp32_ip": ESP_IP,
        "esp32_odometry": esp_data,
        "python_odometry": {"x": robot_x, "y": robot_y, "theta": robot_theta}
    })


if __name__ == "__main__":
    print("=== DDRT LLM Bridge Starting ===")
    print(f"ESP32 target: {ESP_IP}")
    print("Endpoints: /command  /path  /reset  /status")
    app.run(host="0.0.0.0", port=5000, debug=False)
