package frc.robot.hardware.motorcontrol

import com.batterystaple.kmeasure.Seconds
import com.batterystaple.kmeasure.Time
import com.batterystaple.kmeasure.inUnit
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import frc.robot.hardware.interfaces.Encoder
import frc.robot.hardware.interfaces.EncoderMotorController
import frc.robot.hardware.interfaces.MotorConfigurable
import frc.robot.hardware.interfaces.MotorConfiguration
import frc.robot.hardware.interfaces.adaptors.CTREMotorControllerEncoderAdapter

/**
 * Represents a TalonSRX motor controller.
 * Includes everything in the CTRE TalonSRX class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 * @see TalonSRXConfiguration
 */
public open class ChargerTalonSRX(deviceNumber: Int, protected val encoderTicksPerRotation: Int) : WPI_TalonSRX(deviceNumber), EncoderMotorController, MotorConfigurable<TalonSRXConfiguration> {
    final override val encoder: Encoder
        get() = CTREMotorControllerEncoderAdapter(
            ctreMotorController = this,
            pidIndex = 0,
            pulsesPerRotation = encoderTicksPerRotation
        )

    final override fun configure(configuration: TalonSRXConfiguration) {
        configuration.inverted?.let(::setInverted)
        configuration.expiration?.let { expiration = it.inUnit(Seconds) }
        configuration.inverted?.let(::setSafetyEnabled)
    }
}

/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonSRX.
 *
 * @see ChargerTalonSRX
 */
public data class TalonSRXConfiguration(
    var inverted: Boolean? = null,
    var expiration: Time? = null,
    var safetyEnabled: Boolean? = null
) : MotorConfiguration