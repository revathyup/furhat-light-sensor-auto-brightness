package furhatos.app.light.setting

/** Engagement parameters */
const val MAX_NUMBER_OF_USERS = 2 // Max amount of people that Furhat will recognize as users simultaneously
const val DISTANCE_TO_ENGAGE = 1.5 // Min distance for people to be recognised as users
const val SLEEP_DELAY_MS: Int = 10000 // How long to wait with no users before sleeping
const val IDLE_BRIGHTNESS: Double = -75.0 // Brightness to use while idle (before full sleep)
const val LEAVE_GRACE_MS: Int = 2000 // Grace period (ms) before treating everyone as gone
