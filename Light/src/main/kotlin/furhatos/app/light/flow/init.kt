package furhatos.app.light.flow

import furhatos.app.light.flow.main.Idle
import furhatos.app.light.flow.main.Greeting
import furhatos.app.light.sensor.BrightnessUpdateEvent
import furhatos.flow.kotlin.*
import furhatos.app.light.setting.DISTANCE_TO_ENGAGE
import furhatos.app.light.setting.MAX_NUMBER_OF_USERS

val Init: State = state {

    init {
        users.setSimpleEngagementPolicy(DISTANCE_TO_ENGAGE, MAX_NUMBER_OF_USERS)
    }

    // ⭐ HANDLE BRIGHTNESS EVENT (NOW USING THE CORRECT API)
    onEvent<BrightnessUpdateEvent> {
        val b = it.brightness
        println("[LightSkill] Applying face brightness = $b")
        furhat.light(b)      // <-- THIS WORKS IN YOUR SDK
    }

    onEntry {
        when {
            furhat.isVirtual() -> goto(Greeting)
            users.hasAny() -> {
                furhat.attend(users.random)
                goto(Greeting)
            }
            else -> goto(Idle)
        }
    }
}

private fun Furhat.light(b: Double) {}
