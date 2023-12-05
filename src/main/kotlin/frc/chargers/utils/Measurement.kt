package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time
import frc.chargers.wpilibextensions.fpgaTimestamp


/**
 * Represents a measurement read from a generic source, such as an encoder or potentiometer.
 * has the associated timestamp and value.
 *
 * In this class, the input value cannot be nullable.
 */
public class Measurement<T>(
    @JvmField public val value: T,
    timestamp: Time,
): NullableMeasurement<T>(value,timestamp)


/**
 * Represents a measurement read from a generic source, such as an encoder or potentiometer.
 * has the associated timestamp and value:
 * validity is controlled through the nullable value variable.
 */
public open class NullableMeasurement<T>(
    @JvmField public val nullableValue: T?,
    public val timestamp: Time
){

    public val latency: Time = fpgaTimestamp() - timestamp

    public val currentLatency: Time
        get() = fpgaTimestamp() - timestamp

    override fun equals(other: Any?): Boolean =
        other is NullableMeasurement<*> && other.nullableValue == nullableValue

    override fun hashCode(): Int {
        var result = nullableValue?.hashCode() ?: 0
        result = 31 * result + timestamp.hashCode()
        return result
    }



}