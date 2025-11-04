#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include "wifi_config.h"

// Hoverboard communication defines
#define HOVER_SERIAL_BAUD   115200
#define START_FRAME         0xABCD
#define TIME_SEND           100

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
    <p>Sensitivity: <span id="sensitivityValue">1.0</span></p>
    <input type="range" min="0" max="200" value="100" class="slider" id="sensitivitySlider" oninput="updateSensitivity(this.value)">
  </div>
  <div class="controls">
    <p>Use keyboard controls:</p>
    <p>W: Forward, S: Backward, A: Left, D: Right, Space: Stop</p>
    <p>Speed: <span id="speedValue">0</span></p>
    <p>Steer: <span id="steerValue">0</span></p>
    <button class="button" onclick="stopMotors()">STOP</button>
  </div>
  <div class="feedback">
    <p>Left Speed: <span id="leftSpeed">0</span></p>
    <p>Right Speed: <span id="rightSpeed">0</span></p>
    <p>Battery Voltage: <span id="batVoltage">0</span></p>
    <p>Board Temp: <span id="boardTemp">0</span></p>
  </div>
  <script>
    let keys = {};
    let sensitivity = 1.0;
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
          document.getElementById('batVoltage').innerHTML = data.batVoltage;
          document.getElementById('boardTemp').innerHTML = data.boardTemp;
        }
      };
      xhr.open("GET", "/feedback", true);
      xhr.send();
    }, 500);
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
      currentSpeed = request->getParam("speed")->value().toInt();
      currentSteer = request->getParam("steer")->value().toInt();
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  server.on("/feedback", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"leftSpeed\":" + String(Feedback.speedL_meas) + ",";
    json += "\"rightSpeed\":" + String(Feedback.speedR_meas) + ",";
    json += "\"batVoltage\":" + String(Feedback.batVoltage) + ",";
    json += "\"boardTemp\":" + String(Feedback.boardTemp);
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

   // Send commands
   if (iTimeSend > timeNow) return;
   iTimeSend = timeNow + TIME_SEND;
   Send(currentSteer, currentSpeed);
}