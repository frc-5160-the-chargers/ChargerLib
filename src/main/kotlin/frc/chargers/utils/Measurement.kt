package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time
import frc.chargers.wpilibextensions.fpgaTimestamp


/**
 * Represents a measurement read from a generic source, such as an encoder or potentiometer.
 * has the associated timestamp, value, and validity.
 */
public data class Measurement<T>(
    public val value: T,
    public val timestamp: Time,
    public val isValid: Boolean = true
){
    public val latency: Time = fpgaTimestamp() - timestamp

    public val currentLatency: Time
        get() = fpgaTimestamp() - timestamp




    override fun equals(other: Any?): Boolean =
        other is Measurement<*> && other.value == value && other.isValid == isValid

    override fun hashCode(): Int {
        var result = value?.hashCode() ?: 0
        result = 31 * result + timestamp.hashCode()
        result = 31 * result + isValid.hashCode()
        result = 31 * result + latency.hashCode()
        return result
    }
}
