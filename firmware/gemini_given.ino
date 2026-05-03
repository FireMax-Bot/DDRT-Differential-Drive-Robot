// ============================================================
//  DDRT — ESP32 Firmware
//  Fixed: direction-aware braking, missionActive in LLM mode,
//         odometry reset on retrace, brake compensation,
//         rotation factor on returnShortest.
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ===== Wi-Fi Credentials (keep out of version control) =====
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// ===== Hardware Pins =====
#define IN1 26
#define IN2 27
#define IN3 32
#define IN4 33
#define ENA 12
#define ENB 13

WebServer server(80);

// ===== PHYSICAL CALIBRATION =====
// Measure these yourself:
//   velocity      -> drive straight 3 s, measure metres, divide by 3
//   turn_speed_*  -> spin each direction 3 s, measure degrees,
//                    convert to radians, divide by 3
//   Calibrate LEFT and RIGHT turns INDEPENDENTLY — they differ!
int   pwmLeft           = 100;      // ENA duty (0–255)
int   pwmRight          = 96;      // ENB duty (0–255)
float velocity          = 0.266;    // m/s straight-line speed
float turn_speed_left   = 2.2;  // rad/s left-turn angular velocity
float turn_speed_right  = 2.2;  // rad/s right-turn angular velocity  <-- calibrate separately!
int   brakeDelay        = 30;       // ms of active counter-brake pulse

// ===== SOFTWARE TUNING =====
// These are empirically tuned ratios — adjust by running a square test.
// BRAKE_COMP_FACTOR : fraction of brakeDelay to shorten driveTime by
//   (robot still moves a little during the brake pulse)
// RETURN_DIST_FACTOR: forward-distance scale for returnShortest overshoot
// RETURN_ROT_FACTOR : rotation scale for returnShortest over-spin
// STEP_SETTLE_MS    : pause between retrace steps (lets robot settle)
const float BRAKE_COMP_FACTOR  = 0.6f;
const float RETURN_DIST_FACTOR = 0.92f;
const float RETURN_ROT_FACTOR  = 0.97f;
const int   STEP_SETTLE_MS     = 120;

// ===== Odometry State =====
float         x            = 0;
float         y            = 0;
float         theta        = 0;
bool          missionActive = false;
String        currentAction = "stop";
unsigned long lastUpdateTime = 0;
unsigned long moveStartTime  = 0;

// ===== Movement History =====
struct Move {
  String        action;
  unsigned long duration;
};
const int MAX_HISTORY = 200;
Move history[MAX_HISTORY];
int  historyIndex = 0;


// ============================================================
//  LOW-LEVEL MOTOR FUNCTIONS
// ============================================================

void forward() {
  Serial.println("[MOTOR] Forward");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void backward() {
  Serial.println("[MOTOR] Backward");
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void left() {
  Serial.println("[MOTOR] Turn Left");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Motor A forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Motor B backward
}

void right() {
  Serial.println("[MOTOR] Turn Right");
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Motor A backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Motor B forward
}

// ----------------------------------------------------------
//  FIX 1 — Direction-aware active braking
//  Applies a brief counter-impulse that exactly opposes the
//  motion direction before cutting all power.
//  Depends on currentAction being set BEFORE this is called.
// ----------------------------------------------------------
void stopMotor() {
  Serial.print("[MOTOR] Brake for action: "); Serial.println(currentAction);

  if (currentAction == "forward") {
    // Counter: both motors backward
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else if (currentAction == "backward") {
    // Counter: both motors forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (currentAction == "left") {
    // Counter: Motor A backward, Motor B forward
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (currentAction == "right") {
    // Counter: Motor A forward, Motor B backward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  // If already "stop", skip brake pulse

  if (currentAction != "stop") {
    delay(brakeDelay);
  }

  // Full power cut
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  Serial.println("[MOTOR] Full stop.");
}

void normalizeTheta() {
  while (theta >  PI) theta -= 2 * PI;
  while (theta < -PI) theta += 2 * PI;
}


// ============================================================
//  CONTINUOUS ODOMETRY  (runs in loop() for manual moves)
// ============================================================

void updateOdometry() {
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0f;
  lastUpdateTime = now;

  if      (currentAction == "forward")  { x += velocity * cos(theta) * dt; y += velocity * sin(theta) * dt; }
  else if (currentAction == "backward") { x -= velocity * cos(theta) * dt; y -= velocity * sin(theta) * dt; }
  else if (currentAction == "left")     { theta += turn_speed_left  * dt; }
  else if (currentAction == "right")    { theta -= turn_speed_right * dt; }

  normalizeTheta();
}


// ============================================================
//  MANUAL CONTROL ENDPOINTS  (/move  /stop)
// ============================================================

void handleMove() {
  String action = server.arg("action");
  Serial.print("\n[CMD] Continuous move: "); Serial.println(action);

  currentAction = action;
  moveStartTime = millis();
  missionActive = true;

  if      (action == "forward")  forward();
  else if (action == "backward") backward();
  else if (action == "left")     left();
  else if (action == "right")    right();
  else { Serial.println("[ERR] Unknown action!"); }

  server.send(200, "text/plain", "Moving");
}

void handleStop() {
  unsigned long duration = millis() - moveStartTime;
  Serial.print("[CMD] Stop — was: "); Serial.print(currentAction);
  Serial.print(" for "); Serial.print(duration); Serial.println(" ms");

  if (historyIndex < MAX_HISTORY && currentAction != "stop") {
    history[historyIndex].action   = currentAction;
    history[historyIndex].duration = duration;
    historyIndex++;
    Serial.print("History saved. Steps: "); Serial.println(historyIndex);
  }

  stopMotor();          // currentAction still set — direction-aware brake fires
  currentAction = "stop";
  server.send(200, "text/plain", "Stopped");
}


// ============================================================
//  RETURN PATH  (retrace recorded steps in reverse)
// ============================================================

void handleReturnPath() {
  Serial.print("\n[CMD] Return Path — retracing "); Serial.print(historyIndex); Serial.println(" steps");

  for (int i = historyIndex - 1; i >= 0; i--) {
    String        origAction = history[i].action;
    unsigned long duration   = history[i].duration;

    // Invert the original action and store as currentAction
    // so stopMotor() knows which direction to counter-brake.
    if (origAction == "forward")  { currentAction = "backward"; backward(); }
    else if (origAction == "backward") { currentAction = "forward";  forward();  }
    else if (origAction == "left")     { currentAction = "right";    right();    }
    else if (origAction == "right")    { currentAction = "left";     left();     }

    Serial.print("  Step "); Serial.print(i);
    Serial.print(" | inverse of "); Serial.print(origAction);
    Serial.print(" for "); Serial.print(duration); Serial.println(" ms");

    delay(duration);
    stopMotor();
    currentAction = "stop";
    delay(STEP_SETTLE_MS);   // Let robot settle between steps
  }

  // FIX 3 — reset odometry and missionActive after retracing
  historyIndex  = 0;
  x = 0; y = 0; theta = 0;
  missionActive  = false;
  currentAction  = "stop";
  lastUpdateTime = millis();
  Serial.println("Retrace complete. Odometry reset.");
  server.send(200, "text/plain", "Retraced");
}


// ============================================================
//  RETURN SHORTEST  (atan2 + Pythagorean direct path)
// ============================================================

void handleReturnShortest() {
  Serial.println("\n[CMD] Return Shortest");

  if (!missionActive) {
    Serial.println("[ERR] No mission recorded. Ignoring.");
    server.send(200, "text/plain", "No mission recorded");
    return;
  }

  Serial.print("Current pos — X: "); Serial.print(x);
  Serial.print("  Y: "); Serial.print(y);
  Serial.print("  θ: "); Serial.println(theta);

  // ---- 1. Rotate to face origin ----
  float targetAngle = atan2(-y, -x);
  float angleDiff   = targetAngle - theta;
  while (angleDiff >  PI) angleDiff -= 2 * PI;
  while (angleDiff < -PI) angleDiff += 2 * PI;

  float active_turn_speed;
  if (angleDiff > 0) {
    currentAction      = "left";
    active_turn_speed  = turn_speed_left;
    left();
    Serial.println("Decision: turning LEFT to face origin");
  } else {
    currentAction      = "right";
    active_turn_speed  = turn_speed_right;
    right();
    Serial.println("Decision: turning RIGHT to face origin");
  }

  unsigned long rotateTime = (unsigned long)((abs(angleDiff) / active_turn_speed) * 1000.0f);
  Serial.print("Rotate time: "); Serial.print(rotateTime); Serial.println(" ms");

  // FIX 5 — apply RETURN_ROT_FACTOR to the rotation step too
  delay((unsigned long)(rotateTime * RETURN_ROT_FACTOR));
  stopMotor();
  currentAction = "stop";
  delay(STEP_SETTLE_MS);

  // ---- 2. Drive to origin ----
  float         distance = sqrt(x * x + y * y);
  unsigned long moveTime = (unsigned long)((distance / velocity) * 1000.0f);

  Serial.print("Distance: "); Serial.print(distance);
  Serial.print(" m  |  Move time: "); Serial.print(moveTime); Serial.println(" ms");

  currentAction = "forward";
  forward();
  delay((unsigned long)(moveTime * RETURN_DIST_FACTOR));
  stopMotor();

  // ---- 3. Reset everything ----
  x = 0; y = 0; theta = 0;
  missionActive  = false;
  historyIndex   = 0;
  currentAction  = "stop";
  lastUpdateTime = millis();
  Serial.println("Returned to origin. Odometry reset.");
  server.send(200, "text/plain", "Returned shortest");
}


// ============================================================
//  EXACT TURN  (LLM mode — takes degrees)
// ============================================================

void handleTurnExact() {
  String direction      = server.arg("direction");
  float  requestedAngle = server.arg("angle").toFloat();

  Serial.print("\n[CMD] Exact turn — "); Serial.print(direction);
  Serial.print("  "); Serial.print(requestedAngle); Serial.println("°");

  float radiansToTurn = requestedAngle * (PI / 180.0f);

  if (direction == "left") {
    unsigned long turnTime = (unsigned long)((radiansToTurn / turn_speed_left) * 1000.0f);

    // FIX 6 — set currentAction BEFORE calling stopMotor so brake is directional
    currentAction = "left";
    left();
    theta += radiansToTurn;           // Open-loop: update odometry immediately
    normalizeTheta();

    if (historyIndex < MAX_HISTORY) {
      history[historyIndex++] = {"left", turnTime};
    }

    // FIX 2 — mark mission active so returnShortest works from LLM mode
    missionActive = true;

    Serial.print("Turn time: "); Serial.print(turnTime); Serial.println(" ms");
    delay(turnTime);
    stopMotor();

  } else if (direction == "right") {
    unsigned long turnTime = (unsigned long)((radiansToTurn / turn_speed_right) * 1000.0f);

    currentAction = "right";
    right();
    theta -= radiansToTurn;
    normalizeTheta();

    if (historyIndex < MAX_HISTORY) {
      history[historyIndex++] = {"right", turnTime};
    }

    missionActive = true;

    Serial.print("Turn time: "); Serial.print(turnTime); Serial.println(" ms");
    delay(turnTime);
    stopMotor();

  } else {
    Serial.println("[ERR] Bad direction param.");
    server.send(400, "text/plain", "Bad direction. Use 'left' or 'right'.");
    return;
  }

  currentAction  = "stop";
  lastUpdateTime = millis();   // Prevent dt spike on next odometry tick
  Serial.println("Exact turn complete.");
  server.send(200, "text/plain", "Turned " + String(requestedAngle) + " deg " + direction);
}


// ============================================================
//  EXACT MOVE  (LLM mode — takes centimetres)
// ============================================================

void handleMoveExact() {
  String action            = server.arg("action");
  float  targetDistance_cm = server.arg("distance").toFloat();

  Serial.print("\n[CMD] Exact move — "); Serial.print(action);
  Serial.print("  "); Serial.print(targetDistance_cm); Serial.println(" cm");

  float         targetDistance_m = targetDistance_cm / 100.0f;
  unsigned long moveTime         = (unsigned long)((targetDistance_m / velocity) * 1000.0f);

  // FIX 4 — subtract a fraction of brakeDelay from driveTime.
  // The robot keeps moving during the 30 ms brake pulse, so we
  // shorten the drive time to compensate.
  unsigned long brakeComp = (unsigned long)(brakeDelay * BRAKE_COMP_FACTOR);
  unsigned long driveTime = (moveTime > brakeComp) ? (moveTime - brakeComp) : moveTime;

  Serial.print("Drive time (brake-compensated): "); Serial.print(driveTime); Serial.println(" ms");

  // FIX 6 — set currentAction BEFORE motor starts so stopMotor is directional
  // FIX 2 — mark mission active
  currentAction = action;
  missionActive = true;

  if (action == "backward") {
    backward();
    x -= targetDistance_m * cos(theta);
    y -= targetDistance_m * sin(theta);
  } else {
    forward();
    x += targetDistance_m * cos(theta);
    y += targetDistance_m * sin(theta);
  }

  if (historyIndex < MAX_HISTORY) {
    history[historyIndex++] = {action, moveTime};
  }

  delay(driveTime);
  stopMotor();

  currentAction  = "stop";
  lastUpdateTime = millis();
  Serial.println("Exact move complete.");
  server.send(200, "text/plain", "Moved " + String(targetDistance_cm) + " cm");
}


// ============================================================
//  UTILITY ENDPOINTS
// ============================================================

void handlePosition() {
  // Returns current odometry as JSON
  String json = "{";
  json += "\"x\":"     + String(x,     4) + ",";
  json += "\"y\":"     + String(y,     4) + ",";
  json += "\"theta\":" + String(theta, 4) + ",";
  json += "\"mission\":" + String(missionActive ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleClearMemory() {
  historyIndex  = 0;
  x = 0; y = 0; theta = 0;
  missionActive  = false;
  currentAction  = "stop";
  lastUpdateTime = millis();
  Serial.println("\n[CMD] Memory & odometry cleared.");
  server.send(200, "text/plain", "Memory cleared");
}


// ============================================================
//  CALIBRATION ENDPOINTS  (manual bench runs — 3 seconds each)
// ============================================================

void handleCalibrateStraight() {
  Serial.println("\n[CAL] Straight 3 s");
  currentAction = "forward";     
  forward(); delay(3000); stopMotor();
  currentAction = "stop";             
  server.send(200, "text/plain", "Calibration: straight 3 s done");
}

void handleCalibrateTurnLeft() {
  Serial.println("\n[CAL] Left turn 3 s");
  currentAction = "left";
  left(); delay(3000); stopMotor();
  currentAction = "stop";
  server.send(200, "text/plain", "Calibration: left turn 3 s done");
}

void handleCalibrateTurnRight() {
  Serial.println("\n[CAL] Right turn 3 s");
  currentAction = "right";
  right(); delay(3000); stopMotor();
  currentAction = "stop";
  server.send(200, "text/plain", "Calibration: right turn 3 s done");
}

void handleTune() {
  // Read the slider values from the URL
  if (server.hasArg("pwmLeft")) pwmLeft = server.arg("pwmLeft").toInt();
  if (server.hasArg("pwmRight")) pwmRight = server.arg("pwmRight").toInt();
  if (server.hasArg("velocity")) velocity = server.arg("velocity").toFloat();
  if (server.hasArg("turnL")) turn_speed_left = server.arg("turnL").toFloat();
  if (server.hasArg("turnR")) turn_speed_right = server.arg("turnR").toFloat();
  if (server.hasArg("brake")) brakeDelay = server.arg("brake").toInt();

  // Instantly apply the new motor powers!
  analogWrite(ENA, pwmLeft);
  analogWrite(ENB, pwmRight);

  Serial.println("\n[TUNE] Live parameters updated!");
  server.send(200, "text/plain", "Tuning updated");
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== DDRT Booting ===");

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  analogWrite(ENA, pwmLeft);
  analogWrite(ENB, pwmRight);

  // All motors off at start
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

  // Wi-Fi
  Serial.print("Connecting to WiFi: "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nConnected! IP: "); Serial.println(WiFi.localIP());

  // Routes — control
  server.on("/move",         handleMove);
  server.on("/stop",         handleStop);
  server.on("/returnPath",   handleReturnPath);
  server.on("/return",       handleReturnShortest);
  server.on("/moveExact",    handleMoveExact);
  server.on("/turnExact",    handleTurnExact);
  // Routes — utility
  server.on("/position",     handlePosition);
  server.on("/clearMemory",  handleClearMemory);
  // Routes — calibration
  server.on("/calibrate/straight",   handleCalibrateStraight);
  server.on("/calibrate/turn_left",  handleCalibrateTurnLeft);
  server.on("/calibrate/turn_right", handleCalibrateTurnRight);
  server.on("/tune", handleTune);

  server.enableCORS(true);

  server.begin();
  lastUpdateTime = millis();
  Serial.println("Web server started. Awaiting commands.");
  Serial.print("PWM L/R: "); Serial.print(pwmLeft); Serial.print(" / "); Serial.println(pwmRight);
  Serial.print("Velocity: "); Serial.print(velocity); Serial.println(" m/s");
  Serial.print("Turn speed L/R: "); Serial.print(turn_speed_left); Serial.print(" / "); Serial.println(turn_speed_right);
}


// ============================================================
//  LOOP
// ============================================================

void loop() {
  server.handleClient();
  updateOdometry();   // Only updates when currentAction != "stop"
}
