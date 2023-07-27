package frc.chargers.hardware.motorcontrol


import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import edu.wpi.first.wpilibj.motorcontrol.MotorController

import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Represents a motor controller that supports an encoder.
 *
 * Examples of motor controllers supporting encoders are
 * the SparkMax, Talon FX, and Talon SRX.
 */
public interface EncoderMotorController : MotorController {
    public val encoder: Encoder
}

/**
 * Represents a motor controller that supports feedback control on its own processor,
 * in addition to having an encoder.
 */
