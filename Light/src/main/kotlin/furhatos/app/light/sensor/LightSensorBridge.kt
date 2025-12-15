package furhatos.app.light.sensor

import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpServer
import furhatos.event.Event
import furhatos.event.EventSystem
import org.json.JSONObject
import java.net.InetSocketAddress
import java.nio.charset.StandardCharsets
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import kotlin.math.abs

/**
 * Custom event raised inside the skill whenever a new brightness value needs to be sent to Furhat.
 */
class BrightnessUpdateEvent(val brightness: Double) : Event()

/**
 * Very small HTTP bridge that receives lux readings from the ESP8266 (or any other source)
 * and converts them to the BRIGHTNESS parameter used by the face.
 *
 * The endpoint accepts HTTP POST requests on /lux with either:
 *   {"lux": <float>}  - preferred, optionally filtered (bridge smoothing can be disabled)
 *   {"brightness": <float>} - bypasses smoothing if you want to send final brightness
 *   raw float body    - treated as lux
 *
 * Example POST from ESP8266:
 *   curl -X POST http://<furhat-ip>:9914/lux -H "Content-Type: application/json" -d "{\"lux\":42.5}"
 */
object LightSensorBridge {

    private const val SENSOR_PORT = 9914
    private val tuning = BridgeTuning()

    @Volatile
    private var server: HttpServer? = null
    @Volatile
    private var executor: ExecutorService? = null

    // smoothing state, same parameters as the Arduino sketch
    private var hasLux = false
    private var smoothedLux = 0.0
    private var lastBrightness = 0.0
    private var lastSampleMs = 0L
    private var lastForceSendMs = 0L

    @Synchronized
    fun start() {
        if (server != null) {
            println("[LightSensorBridge] Already running on port $SENSOR_PORT")
            return
        }
        val httpServer = HttpServer.create(InetSocketAddress(SENSOR_PORT), 0)
        val executorService = Executors.newSingleThreadExecutor()
        httpServer.createContext("/lux") { exchange -> handleRequest(exchange) }
        httpServer.executor = executorService
        httpServer.start()
        server = httpServer
        executor = executorService
        lastForceSendMs = System.currentTimeMillis()
        println("[LightSensorBridge] Listening on port $SENSOR_PORT (POST /lux)")
        println(
            "[LightSensorBridge] Tuning: interval=${tuning.updateIntervalMs}ms, " +
                "smoothing=${if (tuning.skipSmoothing) "off (trust sensor)" else "alpha=${tuning.luxAlpha}"}, " +
                "luxRange=[${tuning.luxMin}, ${tuning.luxMax}], " +
                "brightnessRange=[${tuning.brightMin}, ${tuning.brightMax}], " +
                "delta>=${tuning.brightDeltaMin}, forceSend=${tuning.forceSendMs}ms"
        )
    }

    @Synchronized
    fun stop() {
        server?.stop(0)
        executor?.shutdownNow()
        server = null
        executor = null
        hasLux = false
        println("[LightSensorBridge] Stopped")
    }

    private fun handleRequest(exchange: HttpExchange) {
        try {
            if (!exchange.requestMethod.equals("POST", ignoreCase = true)) {
                respond(exchange, 405, "Use POST")
                return
            }
            val body = exchange.requestBody.bufferedReader().use { it.readText() }.trim()
            if (body.isEmpty()) {
                respond(exchange, 400, "Body required")
                return
            }
            when (val payload = parsePayload(body)) {
                is SensorPayload.Lux -> {
                    onLux(payload.value)
                    respond(exchange, 204)
                }
                is SensorPayload.Brightness -> {
                    sendBrightness(payload.value, logLux = null)
                    respond(exchange, 204)
                }
                SensorPayload.Invalid -> respond(exchange, 400, "Send {\"lux\":<value>} or {\"brightness\":<value>}")
            }
        } finally {
            exchange.close()
        }
    }

    private fun parsePayload(body: String): SensorPayload {
        body.toDoubleOrNull()?.let { return SensorPayload.Lux(it.toDouble()) }
        return try {
            val json = JSONObject(body)
            when {
                json.has("brightness") -> SensorPayload.Brightness(json.getDouble("brightness"))
                json.has("lux") -> SensorPayload.Lux(json.getDouble("lux"))
                json.has("value") -> SensorPayload.Lux(json.getDouble("value"))
                else -> SensorPayload.Invalid
            }
        } catch (_: Exception) {
            SensorPayload.Invalid
        }
    }

    private fun onLux(rawLux: Double) {
        val now = System.currentTimeMillis()
        if (now - lastSampleMs < tuning.updateIntervalMs) {
            return
        }
        lastSampleMs = now
        val lux = rawLux.coerceAtLeast(0.0)
        val filteredLux = when {
            !hasLux -> lux
            tuning.skipSmoothing -> lux
            else -> smoothedLux + tuning.luxAlpha * (lux - smoothedLux)
        }
        smoothedLux = filteredLux
        hasLux = true

        val brightness = mapRange(
            filteredLux,
            tuning.luxMin,
            tuning.luxMax,
            tuning.brightMin,
            tuning.brightMax
        ).coerceIn(tuning.brightMin, tuning.brightMax)
        val delta = abs(brightness - lastBrightness)
        val allowSend = delta >= tuning.brightDeltaMin || (now - lastForceSendMs) > tuning.forceSendMs
        if (allowSend) {
            sendBrightness(brightness, logLux = filteredLux)
            lastBrightness = brightness
            lastForceSendMs = now
        }
    }

    private fun sendBrightness(value: Double, logLux: Double?) {
        println("[LightSensorBridge] lux=${logLux?.let { String.format("%.1f", it) } ?: "-"} -> brightness=${"%.1f".format(value)}")
        EventSystem.send(BrightnessUpdateEvent(value))
    }

    private fun respond(exchange: HttpExchange, status: Int, message: String = "") {
        val bytes = message.toByteArray(StandardCharsets.UTF_8)
        exchange.sendResponseHeaders(status, bytes.size.toLong())
        exchange.responseBody.use { stream ->
            if (bytes.isNotEmpty()) {
                stream.write(bytes)
            }
        }
    }

    private fun mapRange(x: Double, inMin: Double, inMax: Double, outMin: Double, outMax: Double): Double {
        val clamped = when {
            x < inMin -> inMin
            x > inMax -> inMax
            else -> x
        }
        return (clamped - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
    }

    private sealed interface SensorPayload {
        data class Lux(val value: Double) : SensorPayload
        data class Brightness(val value: Double) : SensorPayload
        object Invalid : SensorPayload
    }

    private data class BridgeTuning(
        val updateIntervalMs: Long = envLong("LIGHT_UPDATE_INTERVAL_MS") ?: 250L,
        val luxAlpha: Double = envDouble("LIGHT_LUX_ALPHA") ?: 0.25,
        val luxMin: Double = envDouble("LIGHT_LUX_MIN") ?: 15.0,
        val luxMax: Double = envDouble("LIGHT_LUX_MAX") ?: 100.0,
        val brightMin: Double = envDouble("LIGHT_BRIGHT_MIN") ?: -76.0,
        val brightMax: Double = envDouble("LIGHT_BRIGHT_MAX") ?: 15.0,
        val brightDeltaMin: Double = envDouble("LIGHT_BRIGHT_DELTA_MIN") ?: 1.5,
        val forceSendMs: Long = envLong("LIGHT_FORCE_SEND_MS") ?: 1500L,
        val skipSmoothing: Boolean = envBool("LIGHT_SKIP_SMOOTHING") ?: false
    )

    private fun envDouble(name: String): Double? {
        val raw = System.getenv(name) ?: System.getProperty(name) ?: return null
        return raw.toDoubleOrNull()
    }

    private fun envLong(name: String): Long? {
        val raw = System.getenv(name) ?: System.getProperty(name) ?: return null
        return raw.toLongOrNull()
    }

    private fun envBool(name: String): Boolean? {
        val raw = System.getenv(name) ?: System.getProperty(name) ?: return null
        return raw.equals("true", ignoreCase = true) || raw == "1" || raw.equals("yes", ignoreCase = true)
    }
}
