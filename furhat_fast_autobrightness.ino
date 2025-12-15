// ESP8266 + Adafruit VEML7700
// Faster Furhat brightness updates (low-latency)

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <math.h>

using namespace websockets;

// ---------------------------
// WiFi Credentials
// ---------------------------
const char *ssid     = "FRHQ";
const char *password = "Thor#2024";

// ---------------------------
// Furhat WebSocket Settings
// ---------------------------
const char *furhatWsURL = "ws://192.168.1.237/api";
String eventSessionId   = "_uu3pgb4_ie63fis5k";

// ---------------------------
// Sensor + WebSocket
// ---------------------------
Adafruit_VEML7700 veml;
WebsocketsClient wsClient;

// ---------------------------
// Update / smoothing params
// ---------------------------
const unsigned long UPDATE_INTERVAL_MS = 250;   // send every 0.25 s
const float LUX_ALPHA = 0.40f;                  // exponential smoothing factor
const float LUX_MIN   = 15.0f;
const float LUX_MAX   = 100.0f;                // tweak to your lighting
const float BRIGHT_MIN = -60.0f;
const float BRIGHT_MAX =  10.0f;
const float BRIGHT_DELTA_MIN = 1.5f;            // don't spam tiny changes

bool  hasLux          = false;
float smoothedLux     = 0.0f;
float lastBrightness  = 0.0f;
unsigned long lastSendMs       = 0;
unsigned long lastForceSendMs  = 0;

// Simple float mapping helper
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// -------------------------------------------------
// Send Brightness JSON to Furhat
// -------------------------------------------------
void sendBrightness(float brightnessValue) {
  StaticJsonDocument<512> doc;
  doc["event_name"] = "furhatos.event.actions.ActionConfigFace";

  JsonArray params = doc.createNestedArray("params");
  JsonObject p = params.createNestedObject();
  p["name"]        = "BRIGHTNESS";
  p["value"]       = brightnessValue;
  p["min"]         = -100;
  p["max"]         =  100;
  p["description"] = "Brightness";

  doc["saveCalibration"] = true;
  doc["event_sessionId"] = eventSessionId;

  String jsonStr;
  serializeJson(doc, jsonStr);

  Serial.print("Sending brightness: ");
  Serial.println(brightnessValue, 1);
  wsClient.send(jsonStr);
}

// -------------------------------------------------
// WebSocket Events
// -------------------------------------------------
void onEventCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("[WS] Connected to Furhat");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("[WS] WebSocket closed");
  }
}

// -------------------------------------------------
// Setup
// -------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Sensor
  Wire.begin();
  Serial.println("Initializing VEML7700...");
  if (!veml.begin()) {
    Serial.println("VEML7700 sensor not found, check wiring!");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("VEML7700 ready");

  // WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP8266 IP: ");
  Serial.println(WiFi.localIP());

  // WebSocket
  wsClient.onEvent(onEventCallback);
  Serial.println("Connecting to Furhat WebSocket...");
  wsClient.connect(furhatWsURL);
  lastForceSendMs = millis();
}

// -------------------------------------------------
// Loop
// -------------------------------------------------
void loop() {
  wsClient.poll();

  unsigned long now = millis();
  if (now - lastSendMs < UPDATE_INTERVAL_MS) {
    return;
  }
  lastSendMs = now;

  float lux = veml.readLux();
  if (!hasLux) {
    smoothedLux = lux;
    hasLux = true;
  } else {
    smoothedLux = smoothedLux + LUX_ALPHA * (lux - smoothedLux);
  }

  float brightness = mapFloat(smoothedLux, LUX_MIN, LUX_MAX, BRIGHT_MIN, BRIGHT_MAX);
  brightness = constrain(brightness, BRIGHT_MIN, BRIGHT_MAX);

  Serial.print("Lux=");
  Serial.print(lux, 1);
  Serial.print(" Smoothed=");
  Serial.print(smoothedLux, 1);
  Serial.print(" Brightness=");
  Serial.println(brightness, 1);

  bool needsSend = fabs(brightness - lastBrightness) >= BRIGHT_DELTA_MIN;
  if (!needsSend && (now - lastForceSendMs) > 2000) {
    // periodically resend even if the change is tiny
    needsSend = true;
  }

  if (needsSend) {
    sendBrightness(brightness);
    lastBrightness   = brightness;
    lastForceSendMs  = now;
  }
}