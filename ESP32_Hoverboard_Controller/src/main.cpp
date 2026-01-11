#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <math.h>
#include "wifi_config.h"

// Hoverboard communication defines
#define HOVER_SERIAL_BAUD   115200
#define START_FRAME         0xABCD
#define TIME_SEND           100

// Odometry constants
// Standard hoverboard wheel: 6.5 inch diameter = 165mm
#define WHEEL_DIAMETER_MM   165.0
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_MM * 3.14159265 / 1000.0)  // in metres
#define WHEEL_BASE_MM       276.0   // Distance between wheel centres
#define WHEEL_BASE_M        (WHEEL_BASE_MM / 1000.0)
// Hoverboard speed feedback scaling
// n_mot is NOT direct RPM - it's (cf_speedCoef/counter)>>4 where cf_speedCoef=10667
// Motor has 15 pole pairs, 6 hall states per e-rev = 90 hall transitions per mech rev
// Empirical calibration: measure 1m travel and adjust until distance matches
#define SPEED_TO_RPM        10.0   // CALIBRATE

// Square mode constants
#define SQUARE_SIDE_LENGTH  1.0     // 1 metre per side
#define TURN_ANGLE_DEG      90.0    // 90 degree turns

// ESC PWM defines
#define ESC_PIN             25        // GPIO for ESC signal
#define ESC_PWM_CHANNEL     0         // LEDC channel
#define ESC_PWM_FREQ        400       // 400Hz for better low-RPM response (most modern ESCs support this)
#define ESC_PWM_RESOLUTION  16        // 16-bit resolution
#define ESC_MIN_PULSE       1000      // Minimum pulse width in microseconds (stopped)
#define ESC_MAX_PULSE       2000      // Maximum pulse width in microseconds (full speed)
#define ESC_RAMP_STEP       2         // Percentage step per ramp interval
#define ESC_RAMP_INTERVAL   20        // Milliseconds between ramp steps

// WiFi credentials are now loaded from wifi_config.h (git-ignored)

// Web server
AsyncWebServer server(80);

// Serial communication with hoverboard
HardwareSerial HoverSerial(2); // Use UART2 (pins 16, 17)

// Global variables
uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// Current commands
int16_t currentSteer = 0;
int16_t currentSpeed = 0;

// ESC control
int escSpeed = 0;         // Current ESC speed 0-100 percentage
int escTargetSpeed = 0;   // Target speed for soft ramp
int escMinThrottle = 15;  // Minimum throttle % to overcome dead zone (adjustable via web)
unsigned long lastRampTime = 0;

// Odometry variables
float odomX = 0.0;              // X position in metres
float odomY = 0.0;              // Y position in metres
float odomTheta = 0.0;          // Heading in radians
float odomDistanceTravelled = 0.0;  // Total distance for current segment
unsigned long lastOdomTime = 0;
float wheelRotationsL = 0.0;    // Accumulated wheel rotations left
float wheelRotationsR = 0.0;    // Accumulated wheel rotations right

// Square mode variables
bool squareModeEnabled = false;
int squareModeSpeed = 150;      // Speed for square mode (0-500)
int squareState = 0;            // 0=idle, 1=driving, 2=turning
int squareSideCount = 0;        // Number of sides completed (0-3)
float squareStartTheta = 0.0;   // Heading at start of turn

// Rotate mode variables
bool rotateModeEnabled = false;
int rotateModeSpeed = 150;      // Speed for rotation (0-500)
float rotateTargetAngle = 0.0;  // Target rotation in radians
float rotateStartTheta = 0.0;   // Heading at start of rotation

// Motor calibration mode variables
bool calibModeEnabled = false;
int calibModeSpeed = 100;       // Speed for calibration
int calibState = 0;             // 0=idle, 1=left motor, 2=right motor
float calibStartRotL = 0.0;     // Starting rotation count left
float calibStartRotR = 0.0;     // Starting rotation count right

// HTML for the web interface
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Hoverboard Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
    .slidecontainer { width: 100%; }
    .slider { -webkit-appearance: none; width: 100%; height: 15px; border-radius: 5px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s;}
    .slider:hover { opacity: 1; }
    .slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 25px; height: 25px; border-radius: 50%; background: #4CAF50; cursor: pointer; }
    .slider::-moz-range-thumb { width: 25px; height: 25px; border-radius: 50%; background: #4CAF50; cursor: pointer; }
    .controls { margin: 20px; }
    .button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
    .button2 {background-color: #555555;}
    .feedback { margin-top: 20px; }
  </style>
</head>
<body>
  <h1>Hoverboard Motor Controller</h1>
  <div class="slidecontainer">
    <p>Sensitivity: <span id="sensitivityValue">0.3</span></p>
    <input type="range" min="0" max="200" value="30" class="slider" id="sensitivitySlider" oninput="updateSensitivity(this.value)">
  </div>
  <div class="controls">
    <p>Use keyboard controls:</p>
    <p>W: Forward, S: Backward, A: Left, D: Right, Space: Stop</p>
    <p>Speed: <span id="speedValue">0</span></p>
    <p>Steer: <span id="steerValue">0</span></p>
    <button class="button" onclick="stopMotors()">STOP</button>
  </div>
  <div class="feedback">
    <h3>Wheel Telemetry</h3>
    <p>Raw Speed L: <span id="leftSpeed">0</span> | R: <span id="rightSpeed">0</span></p>
    <p>Wheel Rotations L: <span id="rotL">0.00</span> | R: <span id="rotR">0.00</span></p>
    <p>Velocity L: <span id="leftVel">0.00</span> m/s | R: <span id="rightVel">0.00</span> m/s</p>
    <p>Position X: <span id="odomX">0.00</span>m | Y: <span id="odomY">0.00</span>m</p>
    <p>Heading: <span id="odomHeading">0.0</span> deg | Distance: <span id="odomDist">0.00</span>m</p>
    <p>Battery: <span id="batVoltage">0</span>V | Temp: <span id="boardTemp">0</span>C</p>
  </div>
  <div class="slidecontainer" style="margin-top: 30px;">
    <h2>Brushless ESC Control</h2>
    <p>ESC Speed: <span id="escValue">0</span>% (Actual: <span id="escActual">0</span>%)</p>
    <input type="range" min="0" max="100" value="0" class="slider" id="escSlider" oninput="updateESC(this.value)">
    <p style="margin-top: 15px;">Min Throttle: <span id="minThrottleValue">15</span>%</p>
    <input type="range" min="0" max="30" value="15" class="slider" id="minThrottleSlider" oninput="updateMinThrottle(this.value)">
    <p style="font-size: 12px; color: #666;">Adjust min throttle to overcome motor dead zone at startup</p>
    <button class="button button2" onclick="stopESC()" style="margin-top: 10px;">STOP ESC</button>
  </div>
  <div class="slidecontainer" style="margin-top: 30px; border-top: 2px solid #4CAF50; padding-top: 20px;">
    <h2>Square Mode (4x4m)</h2>
    <p>Status: <span id="squareStatus" style="font-weight: bold;">Disabled</span></p>
    <p>Side: <span id="squareSide">0</span>/4 | State: <span id="squareState">Idle</span></p>
    <p>Distance: <span id="squareDistance">0.00</span>m | Heading: <span id="squareHeading">0.0</span> deg</p>
    <p style="margin-top: 15px;">Square Speed: <span id="squareSpeedValue">150</span></p>
    <input type="range" min="50" max="400" value="150" class="slider" id="squareSpeedSlider" oninput="updateSquareSpeed(this.value)">
    <p style="font-size: 12px; color: #666;">Adjust speed for square mode (50-400)</p>
    <button class="button" id="squareToggleBtn" onclick="toggleSquareMode()" style="margin-top: 10px;">START SQUARE</button>
    <button class="button button2" onclick="stopSquareMode()" style="margin-top: 10px;">STOP</button>
  </div>
  <div class="slidecontainer" style="margin-top: 30px; border-top: 2px solid #4CAF50; padding-top: 20px;">
    <h2>Rotate Mode</h2>
    <p>Angle: <input type="number" id="rotateAngle" min="0" max="360" value="90" style="width: 60px;"> degrees</p>
    <p>Direction:
      <select id="rotateDir" style="padding: 5px;">
        <option value="1">Clockwise</option>
        <option value="-1">Counter-clockwise</option>
      </select>
    </p>
    <p style="margin-top: 15px;">Rotate Speed: <span id="rotateSpeedValue">150</span></p>
    <input type="range" min="50" max="400" value="150" class="slider" id="rotateSpeedSlider" oninput="updateRotateSpeed(this.value)">
    <p>Status: <span id="rotateStatus">Idle</span></p>
    <button class="button" onclick="startRotate()" style="margin-top: 10px;">ROTATE</button>
    <button class="button button2" onclick="stopRotate()" style="margin-top: 10px;">STOP</button>
  </div>
  <div class="slidecontainer" style="margin-top: 30px; border-top: 2px solid #4CAF50; padding-top: 20px;">
    <h2>Motor Calibration</h2>
    <p style="font-size: 12px; color: #666;">Rotates each motor 1 full revolution for calibration</p>
    <p>Speed: <span id="calibSpeedValue">100</span></p>
    <input type="range" min="50" max="200" value="100" class="slider" id="calibSpeedSlider" oninput="updateCalibSpeed(this.value)">
    <p>Status: <span id="calibStatus">Idle</span></p>
    <button class="button" onclick="startCalib()" style="margin-top: 10px;">START CALIBRATION</button>
    <button class="button button2" onclick="stopCalib()" style="margin-top: 10px;">STOP</button>
  </div>
  <script>
    let keys = {};
    let sensitivity = 0.3;
    document.addEventListener('keydown', (e) => {
      keys[e.key.toLowerCase()] = true;
      updateCommand();
    });
    document.addEventListener('keyup', (e) => {
      keys[e.key.toLowerCase()] = false;
      updateCommand();
    });
    function updateSensitivity(val) {
      sensitivity = parseInt(val) / 100.0;
      document.getElementById('sensitivityValue').innerHTML = sensitivity.toFixed(1);
      updateCommand();
    }
    function updateCommand() {
      let speedDir = 0;
      let steerDir = 0;
      if (keys['w']) speedDir = 1;
      if (keys['s']) speedDir = -1;
      if (keys['a']) steerDir = -1;
      if (keys['d']) steerDir = 1;
      if (keys[' ']) {
        speedDir = 0;
        steerDir = 0;
      }
      let speed = speedDir * 500 * sensitivity;
      let steer = steerDir * 500 * sensitivity;
      document.getElementById('speedValue').innerHTML = speed;
      document.getElementById('steerValue').innerHTML = steer;
      sendCommand(speed, steer);
    }
    function stopMotors() {
      keys = {};
      document.getElementById('speedValue').innerHTML = 0;
      document.getElementById('steerValue').innerHTML = 0;
      sendCommand(0, 0);
    }
    function sendCommand(speed, steer) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/set?speed=" + speed + "&steer=" + steer, true);
      xhr.send();
    }
    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          document.getElementById('leftSpeed').innerHTML = data.leftSpeed;
          document.getElementById('rightSpeed').innerHTML = data.rightSpeed;
          document.getElementById('rotL').innerHTML = data.rotL.toFixed(2);
          document.getElementById('rotR').innerHTML = data.rotR.toFixed(2);
          document.getElementById('leftVel').innerHTML = data.leftVel.toFixed(2);
          document.getElementById('rightVel').innerHTML = data.rightVel.toFixed(2);
          document.getElementById('odomX').innerHTML = data.odomX.toFixed(2);
          document.getElementById('odomY').innerHTML = data.odomY.toFixed(2);
          document.getElementById('odomHeading').innerHTML = data.odomHeading.toFixed(1);
          document.getElementById('odomDist').innerHTML = data.odomDist.toFixed(2);
          document.getElementById('batVoltage').innerHTML = data.batVoltage;
          document.getElementById('boardTemp').innerHTML = data.boardTemp;
        }
      };
      xhr.open("GET", "/feedback", true);
      xhr.send();
    }, 250);
    function updateESC(val) {
      document.getElementById('escValue').innerHTML = val;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/esc?speed=" + val, true);
      xhr.send();
    }
    function updateMinThrottle(val) {
      document.getElementById('minThrottleValue').innerHTML = val;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/escmin?min=" + val, true);
      xhr.send();
    }
    function stopESC() {
      document.getElementById('escSlider').value = 0;
      document.getElementById('escValue').innerHTML = 0;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/esc?speed=0", true);
      xhr.send();
    }
    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          document.getElementById('escActual').innerHTML = data.actual;
        }
      };
      xhr.open("GET", "/escstatus", true);
      xhr.send();
    }, 100);
    let squareEnabled = false;
    function toggleSquareMode() {
      squareEnabled = !squareEnabled;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/square?enable=" + (squareEnabled ? 1 : 0), true);
      xhr.send();
      document.getElementById('squareToggleBtn').innerHTML = squareEnabled ? 'RUNNING...' : 'START SQUARE';
      document.getElementById('squareToggleBtn').style.backgroundColor = squareEnabled ? '#f44336' : '#4CAF50';
    }
    function stopSquareMode() {
      squareEnabled = false;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/square?enable=0", true);
      xhr.send();
      document.getElementById('squareToggleBtn').innerHTML = 'START SQUARE';
      document.getElementById('squareToggleBtn').style.backgroundColor = '#4CAF50';
    }
    function updateSquareSpeed(val) {
      document.getElementById('squareSpeedValue').innerHTML = val;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/squarespeed?speed=" + val, true);
      xhr.send();
    }
    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          document.getElementById('squareStatus').innerHTML = data.enabled ? 'ACTIVE' : 'Disabled';
          document.getElementById('squareStatus').style.color = data.enabled ? '#4CAF50' : '#666';
          document.getElementById('squareSide').innerHTML = data.side;
          var states = ['Idle', 'Driving', 'Turning'];
          document.getElementById('squareState').innerHTML = states[data.state] || 'Unknown';
          document.getElementById('squareDistance').innerHTML = data.distance.toFixed(2);
          document.getElementById('squareHeading').innerHTML = data.heading.toFixed(1);
          if (!data.enabled && squareEnabled) {
            squareEnabled = false;
            document.getElementById('squareToggleBtn').innerHTML = 'START SQUARE';
            document.getElementById('squareToggleBtn').style.backgroundColor = '#4CAF50';
          }
        }
      };
      xhr.open("GET", "/squarestatus", true);
      xhr.send();
    }, 200);
    function startRotate() {
      var angle = document.getElementById('rotateAngle').value;
      var dir = document.getElementById('rotateDir').value;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/rotate?angle=" + angle + "&dir=" + dir, true);
      xhr.send();
      document.getElementById('rotateStatus').innerHTML = 'Rotating...';
    }
    function stopRotate() {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/rotatestop", true);
      xhr.send();
      document.getElementById('rotateStatus').innerHTML = 'Stopped';
    }
    function updateRotateSpeed(val) {
      document.getElementById('rotateSpeedValue').innerHTML = val;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/rotatespeed?speed=" + val, true);
      xhr.send();
    }
    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          if (data.enabled) {
            document.getElementById('rotateStatus').innerHTML = 'Rotating: ' + data.progress.toFixed(1) + ' deg';
          } else if (document.getElementById('rotateStatus').innerHTML.indexOf('Rotating') >= 0) {
            document.getElementById('rotateStatus').innerHTML = 'Done';
          }
        }
      };
      xhr.open("GET", "/rotatestatus", true);
      xhr.send();
    }, 200);
    function startCalib() {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/calib?start=1", true);
      xhr.send();
      document.getElementById('calibStatus').innerHTML = 'Starting...';
    }
    function stopCalib() {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/calibstop", true);
      xhr.send();
      document.getElementById('calibStatus').innerHTML = 'Stopped';
    }
    function updateCalibSpeed(val) {
      document.getElementById('calibSpeedValue').innerHTML = val;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/calibspeed?speed=" + val, true);
      xhr.send();
    }
    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          var states = ['Idle', 'Left motor: ' + data.rotL.toFixed(2) + ' rot', 'Right motor: ' + data.rotR.toFixed(2) + ' rot', 'Done'];
          document.getElementById('calibStatus').innerHTML = states[data.state] || 'Unknown';
        }
      };
      xhr.open("GET", "/calibstatus", true);
      xhr.send();
    }, 200);
  </script>
</body>
</html>
)rawliteral";

void Send(int16_t uSteer, int16_t uSpeed) {
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

void writeESCPulse(int speedPercent) {
  // Apply minimum throttle offset for better starting torque
  int effectiveSpeed = speedPercent;
  if (speedPercent > 0) {
    // Map 1-100% to minThrottle-100% for actual output
    effectiveSpeed = map(speedPercent, 1, 100, escMinThrottle, 100);
  }

  escSpeed = effectiveSpeed;

  // Calculate pulse width: map 0-100% to 1000-2000 microseconds
  int pulseWidth = map(effectiveSpeed, 0, 100, ESC_MIN_PULSE, ESC_MAX_PULSE);

  // Convert pulse width to duty cycle for 16-bit resolution at 400Hz
  // At 400Hz, period is 2500us. Duty = (pulseWidth / 2500) * 65535
  uint32_t duty = (pulseWidth * 65535) / 2500;

  ledcWrite(ESC_PWM_CHANNEL, duty);
}

void setESCTarget(int targetPercent) {
  escTargetSpeed = constrain(targetPercent, 0, 100);
}

void updateESCRamp() {
  // Soft ramp to target speed for smoother starts
  unsigned long now = millis();
  if (now - lastRampTime < ESC_RAMP_INTERVAL) return;
  lastRampTime = now;

  int currentTarget = escTargetSpeed;
  int currentActual = (escSpeed == 0) ? 0 : map(escSpeed, escMinThrottle, 100, 1, 100);

  if (currentActual < currentTarget) {
    // Ramping up
    currentActual += ESC_RAMP_STEP;
    if (currentActual > currentTarget) currentActual = currentTarget;
    writeESCPulse(currentActual);
  } else if (currentActual > currentTarget) {
    // Ramping down (faster for safety)
    currentActual -= ESC_RAMP_STEP * 2;
    if (currentActual < currentTarget) currentActual = currentTarget;
    writeESCPulse(currentActual);
  }
}

// ########################## ODOMETRY ##########################
void updateOdometry() {
  unsigned long now = millis();
  if (lastOdomTime == 0) {
    lastOdomTime = now;
    return;
  }

  float dt = (now - lastOdomTime) / 1000.0;  // Time delta in seconds
  lastOdomTime = now;

  if (dt <= 0 || dt > 1.0) return;  // Skip invalid time deltas

  // Convert speed feedback to m/s
  // speedL_meas and speedR_meas are in internal units (typically ~0.1 RPM per unit)
  float rpmL = Feedback.speedL_meas * SPEED_TO_RPM;
  float rpmR = Feedback.speedR_meas * SPEED_TO_RPM;

  // Convert RPM to m/s: (RPM / 60) * circumference
  float velL = (rpmL / 60.0) * WHEEL_CIRCUMFERENCE;
  float velR = (rpmR / 60.0) * WHEEL_CIRCUMFERENCE;

  // Differential drive kinematics
  float linearVel = (velL + velR) / 2.0;
  float angularVel = (velR - velL) / WHEEL_BASE_M;

  // Update pose
  float deltaTheta = angularVel * dt;
  float deltaX = linearVel * cos(odomTheta + deltaTheta / 2.0) * dt;
  float deltaY = linearVel * sin(odomTheta + deltaTheta / 2.0) * dt;

  odomX += deltaX;
  odomY += deltaY;
  odomTheta += deltaTheta;

  // Normalise theta to [-PI, PI]
  while (odomTheta > 3.14159265) odomTheta -= 2 * 3.14159265;
  while (odomTheta < -3.14159265) odomTheta += 2 * 3.14159265;

  // Update distance travelled for current segment
  odomDistanceTravelled += fabs(linearVel) * dt;

  // Accumulate wheel rotations: RPM / 60 = rotations per second, * dt = rotations
  wheelRotationsL += (rpmL / 60.0) * dt;
  wheelRotationsR += (rpmR / 60.0) * dt;
}

// ########################## SQUARE MODE ##########################
void updateSquareMode() {
  if (!squareModeEnabled) {
    if (squareState != 0) {
      // Just disabled - stop motors
      currentSpeed = 0;
      currentSteer = 0;
      squareState = 0;
    }
    return;
  }

  switch (squareState) {
    case 0:  // Idle - start driving
      odomDistanceTravelled = 0.0;
      squareState = 1;
      squareSideCount = 0;
      Serial.println("Square: Starting side 1");
      break;

    case 1:  // Driving forward
      currentSpeed = -squareModeSpeed;  // Negative for forward direction
      currentSteer = 0;

      if (odomDistanceTravelled >= SQUARE_SIDE_LENGTH) {
        // Reached end of side, start turn
        currentSpeed = 0;
        squareStartTheta = odomTheta;
        squareState = 2;
        Serial.printf("Square: Completed side, starting turn. Distance: %.2fm\n", odomDistanceTravelled);
      }
      break;

    case 2:  // Turning right 90 degrees
      currentSpeed = 0;
      currentSteer = squareModeSpeed / 2;  // Turn right (positive steer)

      // Calculate angle turned (handle wraparound)
      float angleTurned = odomTheta - squareStartTheta;
      // Normalise to [-PI, PI]
      while (angleTurned > 3.14159265) angleTurned -= 2 * 3.14159265;
      while (angleTurned < -3.14159265) angleTurned += 2 * 3.14159265;

      // Check if we've turned 90 degrees (PI/2 radians) - turning right is negative
      if (angleTurned <= -1.5708) {  // -90 degrees in radians
        currentSteer = 0;
        squareSideCount++;
        odomDistanceTravelled = 0.0;

        if (squareSideCount >= 4) {
          // Completed full square
          squareModeEnabled = false;
          squareState = 0;
          Serial.println("Square: Completed full square!");
        } else {
          squareState = 1;
          Serial.printf("Square: Starting side %d\n", squareSideCount + 1);
        }
      }
      break;
  }
}

// ########################## ROTATE MODE ##########################
void updateRotateMode() {
  if (!rotateModeEnabled) {
    return;
  }

  // Calculate angle rotated since start
  float angleRotated = odomTheta - rotateStartTheta;

  // Normalise to [-PI, PI]
  while (angleRotated > 3.14159265) angleRotated -= 2 * 3.14159265;
  while (angleRotated < -3.14159265) angleRotated += 2 * 3.14159265;

  // Check if we've reached target angle
  if (rotateTargetAngle > 0) {
    // Rotating clockwise (negative theta change)
    currentSpeed = 0;
    currentSteer = rotateModeSpeed;
    if (angleRotated <= -rotateTargetAngle) {
      rotateModeEnabled = false;
      currentSteer = 0;
      Serial.println("Rotate: Complete (CW)");
    }
  } else {
    // Rotating counter-clockwise (positive theta change)
    currentSpeed = 0;
    currentSteer = -rotateModeSpeed;
    if (angleRotated >= -rotateTargetAngle) {
      rotateModeEnabled = false;
      currentSteer = 0;
      Serial.println("Rotate: Complete (CCW)");
    }
  }
}

// ########################## CALIBRATION MODE ##########################
void updateCalibMode() {
  if (!calibModeEnabled) {
    return;
  }

  float rotL = wheelRotationsL - calibStartRotL;
  float rotR = wheelRotationsR - calibStartRotR;

  switch (calibState) {
    case 1:  // Left motor - rotate forward
      // Drive forward slowly (both wheels, but we're measuring left)
      currentSpeed = -calibModeSpeed;
      currentSteer = 0;
      if (fabs(rotL) >= 1.0) {
        currentSpeed = 0;
        calibStartRotR = wheelRotationsR;  // Reset right counter
        calibState = 2;
        Serial.printf("Calib: Left done (%.2f rot), starting right\n", rotL);
      }
      break;

    case 2:  // Right motor - rotate forward
      currentSpeed = -calibModeSpeed;
      currentSteer = 0;
      if (fabs(rotR) >= 1.0) {
        currentSpeed = 0;
        calibState = 3;  // Done
        calibModeEnabled = false;
        Serial.printf("Calib: Right done (%.2f rot), calibration complete\n", rotR);
      }
      break;
  }
}

// ########################## RECEIVE ##########################
void Receive() {
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available()) {
    incomingByte = HoverSerial.read();                                   // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  } else {
    return;
  }

  // Copy received data
  if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                        ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
    } else {
      // Non-valid data skipped
    }
    idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

void setup() {
   Serial.begin(115200);
   HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17); // RX=16, TX=17

   // Setup ESC PWM (400Hz for better low-RPM control)
   ledcSetup(ESC_PWM_CHANNEL, ESC_PWM_FREQ, ESC_PWM_RESOLUTION);
   ledcAttachPin(ESC_PIN, ESC_PWM_CHANNEL);
   writeESCPulse(0);  // Start with ESC stopped
   Serial.println("ESC PWM initialised on GPIO 25 at 400Hz");

   // Connect to WiFi
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
   }
   Serial.println("Connected to WiFi");
   Serial.println(WiFi.localIP());

   // Setup OTA
   ArduinoOTA.setHostname("ESP32-HoverboardController");
   ArduinoOTA.setPassword("admin");  // Optional password for OTA updates

   ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
         type = "sketch";
      } else {  // U_SPIFFS
         type = "filesystem";
      }
      Serial.println("Start updating " + type);
   });

   ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
   });

   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
   });

   ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
         Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
         Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
         Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
         Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
         Serial.println("End Failed");
      }
   });

   ArduinoOTA.begin();
   Serial.println("OTA ready");

  // Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("speed") && request->hasParam("steer")) {
      // Disable auto modes when manual control is used
      squareModeEnabled = false;
      squareState = 0;
      rotateModeEnabled = false;
      calibModeEnabled = false;
      calibState = 0;
      // Set manual command
      currentSpeed = request->getParam("speed")->value().toInt();
      currentSteer = request->getParam("steer")->value().toInt();
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  server.on("/feedback", HTTP_GET, [](AsyncWebServerRequest *request){
    // Calculate velocities in m/s
    float rpmL = Feedback.speedL_meas * SPEED_TO_RPM;
    float rpmR = Feedback.speedR_meas * SPEED_TO_RPM;
    float velL = (rpmL / 60.0) * WHEEL_CIRCUMFERENCE;
    float velR = (rpmR / 60.0) * WHEEL_CIRCUMFERENCE;

    String json = "{";
    json += "\"leftSpeed\":" + String(Feedback.speedL_meas) + ",";
    json += "\"rightSpeed\":" + String(Feedback.speedR_meas) + ",";
    json += "\"leftVel\":" + String(velL, 2) + ",";
    json += "\"rightVel\":" + String(velR, 2) + ",";
    json += "\"odomX\":" + String(odomX, 2) + ",";
    json += "\"odomY\":" + String(odomY, 2) + ",";
    json += "\"odomHeading\":" + String(odomTheta * 180.0 / 3.14159265, 1) + ",";
    json += "\"odomDist\":" + String(odomDistanceTravelled, 2) + ",";
    json += "\"rotL\":" + String(wheelRotationsL, 2) + ",";
    json += "\"rotR\":" + String(wheelRotationsR, 2) + ",";
    json += "\"batVoltage\":" + String(Feedback.batVoltage) + ",";
    json += "\"boardTemp\":" + String(Feedback.boardTemp);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/esc", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("speed")) {
      int speed = request->getParam("speed")->value().toInt();
      setESCTarget(speed);
      request->send(200, "text/plain", "ESC target set to " + String(escTargetSpeed) + "%");
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });

  server.on("/escmin", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("min")) {
      escMinThrottle = constrain(request->getParam("min")->value().toInt(), 0, 30);
      request->send(200, "text/plain", "Min throttle set to " + String(escMinThrottle) + "%");
    } else {
      request->send(400, "text/plain", "Missing min parameter");
    }
  });

  server.on("/escstatus", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"actual\":" + String(escSpeed) + ",\"target\":" + String(escTargetSpeed) + "}";
    request->send(200, "application/json", json);
  });

  // Square mode routes
  server.on("/square", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("enable")) {
      int enable = request->getParam("enable")->value().toInt();
      if (enable && !squareModeEnabled) {
        // Starting square mode - reset odometry
        odomX = 0.0;
        odomY = 0.0;
        odomTheta = 0.0;
        odomDistanceTravelled = 0.0;
        squareState = 0;
        squareSideCount = 0;
        lastOdomTime = 0;
      }
      squareModeEnabled = (enable != 0);
      request->send(200, "text/plain", squareModeEnabled ? "Square mode enabled" : "Square mode disabled");
    } else {
      request->send(400, "text/plain", "Missing enable parameter");
    }
  });

  server.on("/squarespeed", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("speed")) {
      squareModeSpeed = constrain(request->getParam("speed")->value().toInt(), 50, 400);
      request->send(200, "text/plain", "Square speed set to " + String(squareModeSpeed));
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });

  server.on("/squarestatus", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"enabled\":" + String(squareModeEnabled ? "true" : "false") + ",";
    json += "\"state\":" + String(squareState) + ",";
    json += "\"side\":" + String(squareSideCount) + ",";
    json += "\"distance\":" + String(odomDistanceTravelled, 2) + ",";
    json += "\"heading\":" + String(odomTheta * 180.0 / 3.14159265, 1) + ",";
    json += "\"x\":" + String(odomX, 2) + ",";
    json += "\"y\":" + String(odomY, 2);
    json += "}";
    request->send(200, "application/json", json);
  });

  // Rotate mode routes
  server.on("/rotate", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("angle") && request->hasParam("dir")) {
      float angleDeg = request->getParam("angle")->value().toFloat();
      int dir = request->getParam("dir")->value().toInt();
      rotateTargetAngle = angleDeg * 3.14159265 / 180.0 * dir;  // Convert to radians with direction
      rotateStartTheta = odomTheta;
      rotateModeEnabled = true;
      request->send(200, "text/plain", "Rotating " + String(angleDeg) + " degrees");
    } else {
      request->send(400, "text/plain", "Missing angle or dir parameter");
    }
  });

  server.on("/rotatestop", HTTP_GET, [](AsyncWebServerRequest *request){
    rotateModeEnabled = false;
    currentSteer = 0;
    currentSpeed = 0;
    request->send(200, "text/plain", "Rotate stopped");
  });

  server.on("/rotatespeed", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("speed")) {
      rotateModeSpeed = constrain(request->getParam("speed")->value().toInt(), 50, 400);
      request->send(200, "text/plain", "Rotate speed set to " + String(rotateModeSpeed));
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });

  server.on("/rotatestatus", HTTP_GET, [](AsyncWebServerRequest *request){
    float angleRotated = odomTheta - rotateStartTheta;
    while (angleRotated > 3.14159265) angleRotated -= 2 * 3.14159265;
    while (angleRotated < -3.14159265) angleRotated += 2 * 3.14159265;
    float progressDeg = fabs(angleRotated) * 180.0 / 3.14159265;

    String json = "{";
    json += "\"enabled\":" + String(rotateModeEnabled ? "true" : "false") + ",";
    json += "\"progress\":" + String(progressDeg, 1);
    json += "}";
    request->send(200, "application/json", json);
  });

  // Calibration mode routes
  server.on("/calib", HTTP_GET, [](AsyncWebServerRequest *request){
    calibStartRotL = wheelRotationsL;
    calibStartRotR = wheelRotationsR;
    calibState = 1;
    calibModeEnabled = true;
    request->send(200, "text/plain", "Calibration started");
  });

  server.on("/calibstop", HTTP_GET, [](AsyncWebServerRequest *request){
    calibModeEnabled = false;
    calibState = 0;
    currentSpeed = 0;
    currentSteer = 0;
    request->send(200, "text/plain", "Calibration stopped");
  });

  server.on("/calibspeed", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("speed")) {
      calibModeSpeed = constrain(request->getParam("speed")->value().toInt(), 50, 200);
      request->send(200, "text/plain", "Calib speed set to " + String(calibModeSpeed));
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });

  server.on("/calibstatus", HTTP_GET, [](AsyncWebServerRequest *request){
    float rotL = wheelRotationsL - calibStartRotL;
    float rotR = wheelRotationsR - calibStartRotR;
    String json = "{";
    json += "\"state\":" + String(calibState) + ",";
    json += "\"rotL\":" + String(fabs(rotL), 2) + ",";
    json += "\"rotR\":" + String(fabs(rotR), 2);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();
}

unsigned long iTimeSend = 0;

void loop(void)
{
   ArduinoOTA.handle();  // Handle OTA updates

   unsigned long timeNow = millis();

   // Check for new received data
   Receive();

   // Update odometry from encoder feedback
   updateOdometry();

   // Update square mode state machine
   updateSquareMode();

   // Update rotate mode
   updateRotateMode();

   // Update calibration mode
   updateCalibMode();

   // Update ESC soft ramp
   updateESCRamp();

   // Send commands
   if (iTimeSend > timeNow) return;
   iTimeSend = timeNow + TIME_SEND;
   Send(currentSteer, currentSpeed);
}