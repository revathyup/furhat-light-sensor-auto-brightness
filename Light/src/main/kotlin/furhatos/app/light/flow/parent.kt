package furhatos.app.light.flow

import furhatos.app.light.flow.main.Idle
import furhatos.app.light.sensor.BrightnessUpdateEvent
import furhatos.event.EventSystem
import furhatos.event.actions.ActionConfigFace
import furhatos.event.responses.ResponseFace
import furhatos.flow.kotlin.*

val Parent: State = state {

    onUserEnter(instant = true) {
        when { // "it" is the user that entered
            furhat.isAttendingUser -> furhat.glance(it) // Glance at new users entering
            !furhat.isAttendingUser -> furhat.attend(it) // Attend user if not attending anyone
        }
    }

    onUserLeave(instant = true) {
        when {
            !users.hasAny() -> { // last user left
                furhat.attendNobody()
                goto(Idle)
            }
            furhat.isAttending(it) -> furhat.attend(users.other) // current user left
            !furhat.isAttending(it) -> furhat.glance(it.head.location) // other user left, just glance
        }
    }

    onEvent<BrightnessUpdateEvent> {
        val brightness = it.brightness.coerceIn(-100.0, 100.0)
        val param = ResponseFace.Param(
            name = "BRIGHTNESS",
            value = brightness,
            min = -100.0,
            max = 100.0,
            description = "Brightness"
        )
        val event = ActionConfigFace.Builder()
            .params(listOf(param))
            .saveCalibration(true)
            .buildEvent()
        EventSystem.send(event)
    }
}
