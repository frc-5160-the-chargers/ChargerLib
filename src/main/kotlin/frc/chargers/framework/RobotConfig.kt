package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds
import org.littletonrobotics.junction.Logger

public data class RobotConfig(
    val tuningMode: Boolean,
    val isReplay: Boolean,
    val extraLoggerConfig: (Logger) -> Unit = {},
    val onError: (Throwable) -> Unit = {},
    val loopPeriod: Time = 0.02.seconds,
    val logToNTWhenFMSAttached: Boolean = true
)