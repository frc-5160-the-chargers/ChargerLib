package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.Timer

/**
 * Represents a measurement read from a generic source, such as an encoder or potentiometer.
 * has the associated timestamp, value, and validity
 */
public data class Measurement<T>(
    public val value: T,
    public val timestamp: Time,
    public val isValid: Boolean,
    private val getCurrentTime: () -> Time = {Timer.getFPGATimestamp().ofUnit(seconds)}
){
    public val latency: Time
        get() = getCurrentTime() - timestamp
}
