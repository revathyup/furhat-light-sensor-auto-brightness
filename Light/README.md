# Light skill
Bridges the Adafruit VEML7700 light sensor (via ESP8266) to Furhat's BRIGHTNESS parameter.

## How it works
* `LightSensorBridge` (see `src/main/kotlin/furhatos/app/light/sensor/LightSensorBridge.kt`) hosts a tiny HTTP server on port **9914**.  
* The ESP8266 sends lux readings (`{"lux": 42.5}`) every ~250 ms to `http://<furhat-ip>:9914/lux`.  
* The skill reuses the same smoothing / clamping constants from the Arduino sketch, maps lux to the `BRIGHTNESS` range, and raises an internal `BrightnessUpdateEvent`.
* `Parent` state listens for those events and dispatches an `ActionConfigFace` so the face brightness is updated with low latency.

## Running the skill
1. Connect the robot and PC to the same network (or run the Virtual Furhat).
2. From this folder run `gradlew shadowJar` and deploy the generated `.skill` package through Furhat Studio / CLI.
3. Start the skill on the robot. A log line similar to `[LightSensorBridge] Listening on port 9914 (POST /lux)` confirms the sensor endpoint is running.

## ESP8266 firmware changes
You can keep almost all of your current Arduino sketch. The only change is where you send the JSON.  
Instead of opening the Furhat WebSocket, send an HTTP POST to the skill:

```cpp
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char *ssid = "FRHQ";
const char *password = "Thor#2024";
const char *endpoint = "http://192.168.1.237:9914/lux"; // replace with the robot/PC IP running the skill

void loop() {
  float lux = veml.readLux();

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    if (http.begin(client, endpoint)) {
      http.addHeader("Content-Type", "application/json");
      String payload = String("{\"lux\":") + String(lux, 1) + "}";
      http.POST(payload); // skill will smooth/map/send brightness
      http.end();
    }
  }

  delay(250);
}
```

If you prefer to keep the ESP8266 responsible for the mapping, post `{"brightness": <value>}` instead; the skill will forward it straight to Furhat.

## Tuning
The smoothing and brightness mapping constants live in `LightSensorBridge`. Defaults map lux into a brightness window of **-76 .. 15** and include modest smoothing (alpha 0.25, 250 ms update, delta 1.5, force send 1500 ms).  
- Override at runtime via env/system props (for Studio add them in the run config): `LIGHT_LUX_MIN`, `LIGHT_LUX_MAX`, `LIGHT_BRIGHT_MIN`, `LIGHT_BRIGHT_MAX`, `LIGHT_BRIGHT_DELTA_MIN`, `LIGHT_FORCE_SEND_MS`, `LIGHT_UPDATE_INTERVAL_MS`, `LIGHT_SKIP_SMOOTHING`, `LIGHT_LUX_ALPHA`.  
- Set `LIGHT_SKIP_SMOOTHING=true` if the ESP already smooths and you want faster reactions; keep it `false` for steadier changes.  
- You can also post `{"brightness": <value>}` to bypass mapping entirely; the skill forwards it straight to the face.
