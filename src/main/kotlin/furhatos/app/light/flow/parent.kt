package furhatos.app.light.flow

import furhatos.app.light.flow.main.Idle
import furhatos.app.light.sensor.BrightnessUpdateEvent
import furhatos.flow.kotlin.*
import furhatos.app.light.setting.LEAVE_GRACE_MS

val Parent: State = state {

    var lastAllGoneAt = 0L

    onUserEnter(instant = true) {
        // Always lock attention to the new user if not already attending them
        lastAllGoneAt = 0L
        if (!furhat.isAttending(it)) {
            furhat.attend(it)
        } else {
            furhat.glance(it)
        }
    }

    onUserLeave(instant = true) {
        when {
            !users.hasAny() -> { // last user left
                furhat.attendNobody()
                lastAllGoneAt = System.currentTimeMillis()
            }
            furhat.isAttending(it) -> furhat.attend(users.other) // current user left
            !furhat.isAttending(it) -> furhat.glance(it.head.location) // other user left, just glance
        }
    }

    onTime(delay = LEAVE_GRACE_MS, repeat = LEAVE_GRACE_MS) {
        if (!users.hasAny()
            && lastAllGoneAt > 0
            && System.currentTimeMillis() - lastAllGoneAt >= LEAVE_GRACE_MS
            && !furhat.isSpeaking
        ) {
            lastAllGoneAt = 0L
            goto(Idle)
        }
    }

    onEvent<BrightnessUpdateEvent> { SleepModeController.onBrightnessUpdate(it.brightness) }
}
