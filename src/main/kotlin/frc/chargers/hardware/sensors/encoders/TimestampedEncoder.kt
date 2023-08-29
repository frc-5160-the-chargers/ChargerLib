package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import frc.chargers.utils.Measurement

/**
 * Represents an encoder that has timestamps
 * corresponding to its angular position and angular velocity measurements.
 */
public interface TimestampedEncoder: Encoder {
    public val timestampedAngularPosition: Measurement<Angle>

    public val timestampedAngularVelocity: Measurement<AngularVelocity>

    override val angularPosition: Angle
        get() = timestampedAngularPosition.value

    override val angularVelocity: AngularVelocity
        get() = timestampedAngularVelocity.value
}