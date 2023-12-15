package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import frc.chargers.utils.QuantityMeasurement

/**
 * Represents an encoder that has timestamps
 * corresponding to its angular position and angular velocity measurements.
 */
public interface TimestampedEncoder: Encoder {
    public val timestampedAngularPosition: QuantityMeasurement<AngleDimension>

    public val timestampedAngularVelocity: QuantityMeasurement<AngularVelocityDimension>

    override val angularPosition: Angle
        get() = timestampedAngularPosition.value

    override val angularVelocity: AngularVelocity
        get() = timestampedAngularVelocity.value
}