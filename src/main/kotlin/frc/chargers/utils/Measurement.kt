package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time
import frc.chargers.wpilibextensions.Timer


/**
 * Represents a measurement read from a generic source, such as an encoder or potentiometer.
 * has the associated timestamp, value, and validity
 */
public data class Measurement<T>(
    public val value: T,
    public val timestamp: Time,
    public val isValid: Boolean
){
    public val latency: Time = Timer.getFPGATimestamp() - timestamp

    public val currentLatency: Time
        get() = Timer.getFPGATimestamp() - timestamp
}
