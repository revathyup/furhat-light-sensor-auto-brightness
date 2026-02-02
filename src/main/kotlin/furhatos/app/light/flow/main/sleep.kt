package furhatos.app.light.flow.main

import furhatos.app.light.flow.Parent
import furhatos.app.light.flow.SleepModeController
import furhatos.flow.kotlin.State
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.onUserEnter
import furhatos.flow.kotlin.onUserLeave
import furhatos.flow.kotlin.state

val Sleep: State = state(Parent) {
    onEntry {
        furhat.attendNobody()
        SleepModeController.enter(furhat)
    }

    onUserEnter {
        furhat.attend(it)
        SleepModeController.exit(furhat)
        goto(Greeting)
    }

    onUserLeave {
        SleepModeController.exit(furhat)
    }
}

