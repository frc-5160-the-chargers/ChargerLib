package frc.robot.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.robot.hardware.sensors.encoders.Encoder

/**
 * Represents a motor controller that supports an encoder.
 *
 * Examples of motor controllers supporting encoders are
 * the SparkMax, Talon FX, and Talon SRX.
 */
public interface EncoderMotorController : MotorController {
    public val encoder: Encoder
}