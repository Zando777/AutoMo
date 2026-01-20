#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include "wifi_config.h"

// Configuration
#define TCP_PORT            8880
#define HOVER_SERIAL_BAUD   115200
#define START_FRAME         0xABCD

// Serial communication with hoverboard
HardwareSerial HoverSerial(2);  // UART2: RX=16, TX=17

// TCP Server
AsyncServer tcpServer(TCP_PORT);
AsyncClient* tcpClient = nullptr;

// Web server for status
AsyncWebServer webServer(80);

// Statistics
volatile uint32_t bytesToHoverboard = 0;
volatile uint32_t bytesFromHoverboard = 0;
volatile uint32_t tcpConnections = 0;
volatile bool clientConnected = false;

// Ring buffer for UART -> TCP
#define UART_BUFFER_SIZE 512
uint8_t uartBuffer[UART_BUFFER_SIZE];
volatile uint16_t uartBufferHead = 0;
volatile uint16_t uartBufferTail = 0;

// Hoverboard feedback parsing (for debug display)
typedef struct {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  wheelR_cnt;
    int16_t  wheelL_cnt;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;

SerialFeedback lastFeedback;
bool feedbackValid = false;

// Feedback parsing state
uint8_t feedbackIdx = 0;
uint8_t feedbackBuf[sizeof(SerialFeedback)];
uint8_t prevByte = 0;

// HTML for status page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 ROS Bridge</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0 auto; padding: 20px; max-width: 600px; }
    .status { padding: 15px; margin: 10px; border-radius: 8px; }
    .connected { background: #d4edda; color: #155724; }
    .disconnected { background: #f8d7da; color: #721c24; }
    .info { background: #e2e3e5; color: #383d41; }
    table { width: 100%; border-collapse: collapse; margin: 15px 0; }
    td, th { padding: 10px; border: 1px solid #ddd; text-align: left; }
    th { background: #f5f5f5; }
    h1 { color: #333; }
    h3 { color: #666; margin-top: 25px; }
    .mono { font-family: monospace; }
  </style>
</head>
<body>
  <h1>ESP32 ROS Bridge</h1>
  <p>WiFi-to-Serial bridge for ROS hoverboard-driver</p>

  <h3>Connection Status</h3>
  <div id="wifiStatus" class="status connected">WiFi: Connected</div>
  <div id="tcpStatus" class="status disconnected">TCP Client: Disconnected</div>

  <h3>Statistics</h3>
  <table>
    <tr><th>Metric</th><th>Value</th></tr>
    <tr><td>Bytes to Hoverboard</td><td id="txBytes" class="mono">0</td></tr>
    <tr><td>Bytes from Hoverboard</td><td id="rxBytes" class="mono">0</td></tr>
    <tr><td>TCP Connections</td><td id="connCount" class="mono">0</td></tr>
  </table>

  <h3>Hoverboard Feedback</h3>
  <div id="feedbackStatus" class="status info">Waiting for data...</div>
  <table id="feedbackTable" style="display:none;">
    <tr><td>Speed L</td><td id="speedL" class="mono">0</td></tr>
    <tr><td>Speed R</td><td id="speedR" class="mono">0</td></tr>
    <tr><td>Encoder L</td><td id="encL" class="mono">0</td></tr>
    <tr><td>Encoder R</td><td id="encR" class="mono">0</td></tr>
    <tr><td>Battery</td><td id="bat" class="mono">0 V</td></tr>
    <tr><td>Temperature</td><td id="temp" class="mono">0 C</td></tr>
  </table>

  <h3>Usage</h3>
  <div class="status info" style="text-align: left;">
    <p class="mono">socat pty,link=/tmp/hoverboard,raw tcp:<span id="espIP">ESP_IP</span>:8880</p>
    <p>Then configure ROS driver with port: /tmp/hoverboard</p>
  </div>

  <script>
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('txBytes').textContent = data.txBytes;
          document.getElementById('rxBytes').textContent = data.rxBytes;
          document.getElementById('connCount').textContent = data.connections;

          var tcpDiv = document.getElementById('tcpStatus');
          if (data.clientConnected) {
            tcpDiv.textContent = 'TCP Client: Connected';
            tcpDiv.className = 'status connected';
          } else {
            tcpDiv.textContent = 'TCP Client: Disconnected';
            tcpDiv.className = 'status disconnected';
          }

          if (data.feedbackValid) {
            document.getElementById('feedbackStatus').style.display = 'none';
            document.getElementById('feedbackTable').style.display = 'table';
            document.getElementById('speedL').textContent = data.speedL;
            document.getElementById('speedR').textContent = data.speedR;
            document.getElementById('encL').textContent = data.encL;
            document.getElementById('encR').textContent = data.encR;
            document.getElementById('bat').textContent = (data.bat / 100.0).toFixed(1) + ' V';
            document.getElementById('temp').textContent = (data.temp / 10.0).toFixed(1) + ' C';
          }

          document.getElementById('espIP').textContent = data.ip;
        })
        .catch(e => console.error('Status fetch failed:', e));
    }

    updateStatus();
    setInterval(updateStatus, 500);
  </script>
</body>
</html>
)rawliteral";

// Parse incoming UART bytes for feedback struct (non-blocking)
void parseFeedbackByte(uint8_t b) {
    uint16_t startFrame = ((uint16_t)b << 8) | prevByte;

    if (startFrame == START_FRAME) {
        feedbackBuf[0] = prevByte;
        feedbackBuf[1] = b;
        feedbackIdx = 2;
    } else if (feedbackIdx >= 2 && feedbackIdx < sizeof(SerialFeedback)) {
        feedbackBuf[feedbackIdx++] = b;
    }

    if (feedbackIdx == sizeof(SerialFeedback)) {
        SerialFeedback* fb = (SerialFeedback*)feedbackBuf;
        uint16_t checksum = fb->start ^ fb->cmd1 ^ fb->cmd2 ^ fb->speedR_meas ^
                           fb->speedL_meas ^ fb->wheelR_cnt ^ fb->wheelL_cnt ^
                           fb->batVoltage ^ fb->boardTemp ^ fb->cmdLed;

        if (fb->start == START_FRAME && checksum == fb->checksum) {
            memcpy(&lastFeedback, fb, sizeof(SerialFeedback));
            feedbackValid = true;
        }
        feedbackIdx = 0;
    }

    prevByte = b;
}

// Handle new TCP client connection
void handleNewClient(void* arg, AsyncClient* client) {
    Serial.printf("TCP client connected from %s\n", client->remoteIP().toString().c_str());

    // Disconnect existing client if any
    if (tcpClient != nullptr && tcpClient->connected()) {
        Serial.println("Disconnecting previous client");
        tcpClient->close();
    }

    tcpClient = client;
    clientConnected = true;
    tcpConnections++;

    // Set up client callbacks
    client->onData([](void* arg, AsyncClient* c, void* data, size_t len) {
        // TCP -> UART (to hoverboard)
        uint8_t* bytes = (uint8_t*)data;
        HoverSerial.write(bytes, len);
        bytesToHoverboard += len;
    }, nullptr);

    client->onDisconnect([](void* arg, AsyncClient* c) {
        Serial.println("TCP client disconnected");
        clientConnected = false;
        if (tcpClient == c) {
            tcpClient = nullptr;
        }
    }, nullptr);

    client->onError([](void* arg, AsyncClient* c, int8_t error) {
        Serial.printf("TCP error: %d\n", error);
    }, nullptr);
}

void setupWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.config(staticIP, gateway, subnet, dns);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("WiFi connection failed!");
    }
}

void setupOTA() {
    ArduinoOTA.setHostname("ESP32-ROS-Bridge");
    ArduinoOTA.setPassword("admin");

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("OTA Start: " + type);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA ready");
}

void setupWebServer() {
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
    });

    webServer.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"txBytes\":" + String(bytesToHoverboard) + ",";
        json += "\"rxBytes\":" + String(bytesFromHoverboard) + ",";
        json += "\"connections\":" + String(tcpConnections) + ",";
        json += "\"clientConnected\":" + String(clientConnected ? "true" : "false") + ",";
        json += "\"feedbackValid\":" + String(feedbackValid ? "true" : "false") + ",";
        json += "\"speedL\":" + String(lastFeedback.speedL_meas) + ",";
        json += "\"speedR\":" + String(lastFeedback.speedR_meas) + ",";
        json += "\"encL\":" + String(lastFeedback.wheelL_cnt) + ",";
        json += "\"encR\":" + String(lastFeedback.wheelR_cnt) + ",";
        json += "\"bat\":" + String(lastFeedback.batVoltage) + ",";
        json += "\"temp\":" + String(lastFeedback.boardTemp) + ",";
        json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
        json += "}";
        request->send(200, "application/json", json);
    });

    webServer.begin();
    Serial.println("Web server started on port 80");
}

void setupTCPServer() {
    tcpServer.onClient(&handleNewClient, nullptr);
    tcpServer.begin();
    Serial.printf("TCP server started on port %d\n", TCP_PORT);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\nESP32 ROS Bridge starting...");

    // Initialise UART to hoverboard
    HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17);
    Serial.println("UART to hoverboard initialised (GPIO16 RX, GPIO17 TX)");

    setupWiFi();
    setupOTA();
    setupWebServer();
    setupTCPServer();

    Serial.println("Bridge ready!");
    Serial.printf("Connect via: socat pty,link=/tmp/hoverboard,raw tcp:%s:%d\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT);
}

void loop() {
    ArduinoOTA.handle();

    // UART -> TCP (from hoverboard to ROS)
    while (HoverSerial.available()) {
        uint8_t b = HoverSerial.read();
        bytesFromHoverboard++;

        // Parse for debug display
        parseFeedbackByte(b);

        // Forward to TCP client if connected
        if (tcpClient != nullptr && tcpClient->connected() && tcpClient->canSend()) {
            tcpClient->write((const char*)&b, 1);
        }
    }

    // Small yield to prevent watchdog
    yield();
}
