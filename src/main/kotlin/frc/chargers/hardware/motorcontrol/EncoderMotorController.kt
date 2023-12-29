package frc.chargers.hardware.motorcontrol


import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder

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
 * An interface that represents an [EncoderMotorController]
 * that can measure it's recorded temperature, applied current, and applied voltage,
 * as well as run closed loop control on the motor itself.
 */
public interface SmartEncoderMotorController: EncoderMotorController, TemperatureProvider, CurrentProvider, VoltageProvider{

    /**
     * Sets the angular velocity of the motor.
     */
    public fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
    )

    /**
     * Sets the position of the motor using closed loop control,
     * utilizing the output of an optional absolute encoder.
     */
    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        absoluteEncoder: PositionEncoder? = null,
        extraVoltage: Voltage = Voltage(0.0)
    )


}

