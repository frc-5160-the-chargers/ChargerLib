package frc.chargers.framework

import org.littletonrobotics.junction.Logger

public data class RobotConfig(
    val tuningMode: Boolean,
    val isReplay: Boolean,
    val extraLoggerConfig: (Logger) -> Unit = {},
    val onError: (Throwable) -> Unit = {}
)