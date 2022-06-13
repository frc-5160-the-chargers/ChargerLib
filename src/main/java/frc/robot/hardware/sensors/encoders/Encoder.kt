package frc.robot.hardware.sensors.encoders

import com.batterystaple.kmeasure.Angle
import com.batterystaple.kmeasure.AngularVelocity

/**
 * Represents a generic encoder.
 *
 * An encoder is a device affixed to a motor that measures its position.
 * From this, the angle and angular velocity of the motor can be determined.
 */
public interface Encoder {
    public val angularPosition: Angle
    public val angularVelocity: AngularVelocity
}