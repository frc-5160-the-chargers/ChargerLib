package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds

public data class RobotConfig(
    val tuningMode: Boolean,
    val isReplay: Boolean,
    val extraLoggerConfig: () -> Unit = {},
    val onError: (Throwable) -> Unit = {},
    val loopPeriod: Time = 0.02.seconds
)