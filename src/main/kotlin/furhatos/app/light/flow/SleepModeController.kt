package furhatos.app.light.flow

import furhatos.app.light.setting.IDLE_BRIGHTNESS
import furhatos.event.EventSystem
import furhatos.event.actions.ActionConfigFace
import furhatos.event.responses.ResponseFace
import furhatos.flow.kotlin.Furhat
import furhatos.gestures.Gestures
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/**
 * Centralizes sleep state management so that other parts of the skill
 * can check whether the robot should accept external brightness updates.
 */
object SleepModeController {
    private val sleeping = AtomicBoolean(false)
    private val idleDimming = AtomicBoolean(false)
    private val lastBrightness = AtomicReference<Double?>(null)
    private const val SLEEP_BRIGHTNESS = -75.0

    val isSleeping: Boolean
        get() = sleeping.get()

    fun enter(furhat: Furhat) {
        if (!sleeping.compareAndSet(false, true)) {
            return
        }
        idleDimming.set(false)
        applyBrightness(SLEEP_BRIGHTNESS, saveCalibration = false)
        furhat.gesture(Gestures.CloseEyes, async = true)
    }

    fun exit(furhat: Furhat) {
        if (!sleeping.compareAndSet(true, false)) {
            return
        }
        furhat.gesture(Gestures.OpenEyes, async = true)
        restoreLastBrightness()
    }

    fun enterIdleDim() {
        if (idleDimming.compareAndSet(false, true)) {
            applyBrightness(IDLE_BRIGHTNESS, saveCalibration = false)
        }
    }

    fun exitIdleDim() {
        if (idleDimming.compareAndSet(true, false)) {
            restoreLastBrightness()
        }
    }

    /**
     * Handles a brightness update from the sensor bridge. While sleeping we only cache
     * the value; once awake we immediately apply the latest cached brightness unless
     * we are intentionally dimming in idle.
     */
    fun onBrightnessUpdate(brightness: Double, saveCalibration: Boolean = true) {
        val clamped = brightness.coerceIn(-100.0, 100.0)
        lastBrightness.set(clamped)
        if (!isSleeping && !idleDimming.get()) {
            applyBrightness(clamped, saveCalibration)
        }
    }

    private fun restoreLastBrightness() {
        val cached = lastBrightness.get() ?: return
        applyBrightness(cached)
    }

    private fun applyBrightness(brightness: Double, saveCalibration: Boolean = true) {
        val clamped = brightness.coerceIn(-100.0, 100.0)
        val param = ResponseFace.Param(
            name = "BRIGHTNESS",
            value = clamped,
            min = -100.0,
            max = 100.0,
            description = "Brightness"
        )
        val event = ActionConfigFace.Builder()
            .params(listOf(param))
            .saveCalibration(saveCalibration)
            .buildEvent()
        EventSystem.send(event)
    }
}
