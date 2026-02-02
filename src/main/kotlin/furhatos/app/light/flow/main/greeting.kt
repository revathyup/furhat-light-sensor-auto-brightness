package furhatos.app.light.flow.main

import furhatos.app.light.flow.Parent
import furhatos.flow.kotlin.State
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.onResponse
import furhatos.flow.kotlin.state
import furhatos.nlu.common.No
import furhatos.nlu.common.Yes

val Greeting: State = state(Parent) {
    onEntry {
        // Say fully first to avoid barge-in cutting the utterance, then listen for Yes/No
        furhat.say("Hello, I'm Furhat. Nice to meet you.")
        furhat.listen()
    }

    onResponse<Yes> {
        furhat.say("Nice to meet you.")
    }

    onResponse<No> {
        furhat.say("No problem.")
    }

}
