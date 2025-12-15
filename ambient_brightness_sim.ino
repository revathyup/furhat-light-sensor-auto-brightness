// ESP8266 + Adafruit VEML7700
// Auto-brightness controller for Furhat over WebSocket

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

// ---------------------------
// WiFi credentials
// ---------------------------
const char *ssid     = "FRHQ";       // your Wi-Fi SSID
const char *password = "Thor#2024";  // your Wi-Fi password

// ---------------------------
// Furhat WebSocket settings
// ---------------------------
// Use the robot's IP and /api path, e.g. ws://192.168.1.237/api
const char *furhatWsURL = "ws://192.168.1.237/api";

// IMPORTANT: update this with the current session id
// from the Furhat Real-time API (it looks like _xxxx_xxxxx)
String eventSessionId = "_mcdr2dd_ra936yjqk";

// ---------------------------
// Objects
// ---------------------------
WebsocketsClient wsClient;
Adafruit_VEML7700 veml;

// Flags / state
bool vemlOk   = false;
bool enabled  = false;  // start OFF; press '1' in Serial Monitor

// Lux smoothing
bool  hasLux      = false;
float smoothedLux = 0.0f;
const float LUX_ALPHA = 0.6f;  // 0..1, higher = more responsive

// Office environment lux & brightness ranges
const float MIN_LUX     = 10.0f;   // very dim
const float MAX_LUX     = 500.0f;  // bright office
const float MIN_BRIGHT  = -80.0f;
const float MAX_BRIGHT  =  80.0f;

// Send interval
unsigned long lastSendMs = 0;
const unsigned long SEND_INTERVAL_MS = 400; // faster updates

struct FaceParam {
  const char *name;
  float value;
  float minValue;
  float maxValue;
  const char *description;
};

FaceParam faceParams[] = {
  {"POSITION_X",   0.8f,   -50.0f,  50.0f,  "Left/Right"},
  {"POSITION_Z",  -5.7f,   -50.0f,  50.0f,  "Up/Down"},
  {"SCALING_X",    1.646f,  0.5f,    2.5f,  "Horizontal scaling"},
  {"SCALING_Z",    1.488f,  0.5f,    2.5f,  "Vertical scaling"},
  {"ROTATION_Z",   0.011f, -0.3f,    0.3f,  "Roll"},
  {"LEFT_EYE_X",  -1.8f,  -20.0f,   20.0f,  "Left eye"},
  {"RIGHT_EYE_X", -1.6f,  -20.0f,   20.0f,  "Right eye"},
  {"CONTRAST",     8.4f,  -100.0f, 100.0f,  "Contrast"},
  {"SATURATION", -73.8f,  -100.0f, 100.0f,  "Saturation"},
  {"BRIGHTNESS",  24.6f,  -100.0f, 100.0f,  "Brightness"}
};
const size_t kFaceParamCount = sizeof(faceParams) / sizeof(faceParams[0]);

// Current brightness value
float brightnessValue = 0.0f;

// -------------------------------------------------
// Map lux to Furhat brightness [-80..80]
// -------------------------------------------------
float mapLuxToBrightness(float lux) {
  if (lux < MIN_LUX) lux = MIN_LUX;
  if (lux > MAX_LUX) lux = MAX_LUX;

  float t = (lux - MIN_LUX) / (MAX_LUX - MIN_LUX); // 0..1
  return MIN_BRIGHT + t * (MAX_BRIGHT - MIN_BRIGHT);
}

// -------------------------------------------------
// Create and send Furhat BRIGHTNESS event
// -------------------------------------------------
void sendBrightness() {
  if (!wsClient.available()) {
    Serial.println("WS not available, skip sendBrightness()");
    return;
  }

  StaticJsonDocument<512> doc;
  doc["event_name"]      = "furhatos.event.actions.ActionConfigFace";
  doc["event_sessionId"] = eventSessionId;
  doc["saveCalibration"] = false;

  // Update brightness value inside the parameter array
  faceParams[9].value = brightnessValue;

  JsonArray params = doc.createNestedArray("params");
  for (size_t i = 0; i < kFaceParamCount; ++i) {
    JsonObject p = params.createNestedObject();
    p["name"]        = faceParams[i].name;
    p["value"]       = faceParams[i].value;
    p["min"]         = faceParams[i].minValue;
    p["max"]         = faceParams[i].maxValue;
    p["description"] = faceParams[i].description;
  }

  String jsonStr;
  serializeJson(doc, jsonStr);

  Serial.println("Sending brightness JSON:");
  Serial.println(jsonStr);

  wsClient.send(jsonStr);
}

// -------------------------------------------------
// WebSocket callbacks
// -------------------------------------------------
void onMessageCallback(WebsocketsMessage message) {
  // Uncomment for debug
  // Serial.print("[WS RX] ");
  // Serial.println(message.data());
}

void onEventCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("[WS] Connection opened");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("[WS] Connection closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("[WS] Got Ping");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("[WS] Got Pong");
  }
}

// -------------------------------------------------
// Setup
// -------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("ESP8266 IP: ");
  Serial.println(WiFi.localIP());

  // I2C + VEML7700
  // On ESP8266 (NodeMCU / Wemos): SDA = D2 (GPIO4), SCL = D1 (GPIO5)
  Wire.begin(D2, D1);
  if (!veml.begin()) {
    Serial.println("Error: VEML7700 not found. Check wiring.");
    vemlOk = false;
  } else {
    Serial.println("VEML7700 found.");
    vemlOk = true;
    veml.setGain(VEML7700_GAIN_1);
    veml.setIntegrationTime(VEML7700_IT_100MS);
  }

  // WebSocket setup
  wsClient.onMessage(onMessageCallback);
  wsClient.onEvent(onEventCallback);

  Serial.println("Connecting to Furhat WebSocket...");
  if (!wsClient.connect(furhatWsURL)) {
    Serial.println("Failed to connect to Furhat WebSocket!");
  } else {
    Serial.println("WebSocket connected.");
  }

  Serial.println("Type '1' + Enter in Serial Monitor to ENABLE auto brightness");
  Serial.println("Type '0' + Enter in Serial Monitor to DISABLE");
}

// -------------------------------------------------
// Loop
// -------------------------------------------------
void loop() {
  // Keep WebSocket alive
  wsClient.poll();

  // Manual enable/disable from Serial Monitor
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0') {
      enabled = false;
      Serial.println("Auto brightness OFF");
    } else if (c == '1') {
      enabled = true;
      Serial.println("Auto brightness ON");
    }
  }

  if (!enabled) {
    return;  // do nothing when disabled
  }

  // If sensor not OK, don't try to send brightness
  if (!vemlOk) {
    static unsigned long lastMsg = 0;
    unsigned long now = millis();
    if (now - lastMsg > 3000) {
      lastMsg = now;
      Serial.println("VEML7700 not OK, cannot send brightness.");
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastSendMs < SEND_INTERVAL_MS) {
    return;
  }
  lastSendMs = now;

  // Read lux
  float lux = veml.readLux();
  if (!hasLux) {
    smoothedLux = lux;
    hasLux = true;
  } else {
    smoothedLux = (1.0f - LUX_ALPHA) * smoothedLux + LUX_ALPHA * lux;
  }

  brightnessValue = mapLuxToBrightness(smoothedLux);

  Serial.print("Lux=");
  Serial.print(lux, 1);
  Serial.print("  SmoothedLux=");
  Serial.print(smoothedLux, 1);
  Serial.print("  Brightness=");
  Serial.println(brightnessValue, 1);

  sendBrightness();
}
