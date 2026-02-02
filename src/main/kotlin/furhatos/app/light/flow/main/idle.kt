package furhatos.app.light.flow.main

import furhatos.app.light.flow.Parent
import furhatos.app.light.flow.SleepModeController
import furhatos.app.light.setting.SLEEP_DELAY_MS
import furhatos.flow.kotlin.State
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.onUserEnter
import furhatos.flow.kotlin.state
import furhatos.flow.kotlin.users
import furhatos.records.User

val Idle: State = state(Parent) {
    onEntry {
        furhat.attendNobody()
        SleepModeController.exitIdleDim()
        SleepModeController.exit(furhat)
        SleepModeController.enterIdleDim()
    }

    onUserEnter {
        furhat.attend(it)
        SleepModeController.exitIdleDim()
        goto(Greeting)
    }

    onTime(delay = SLEEP_DELAY_MS, cond = { users.count < 1 }) {
        goto(Sleep)
    }
}
