package furhatos.app.light

import furhatos.app.light.flow.Init
import furhatos.app.light.sensor.LightSensorBridge
import furhatos.flow.kotlin.Flow
import furhatos.skills.Skill

class LightSkill : Skill() {
    private val shutdownHook = Thread { LightSensorBridge.stop() }

    override fun start() {
        LightSensorBridge.start()
        Runtime.getRuntime().addShutdownHook(shutdownHook)
        Flow().run(Init)
    }
}

fun main(args: Array<String>) {
    Skill.main(args)
}
