package frc.chargers.hardware.motorcontrol


import edu.wpi.first.wpilibj.motorcontrol.MotorController
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
