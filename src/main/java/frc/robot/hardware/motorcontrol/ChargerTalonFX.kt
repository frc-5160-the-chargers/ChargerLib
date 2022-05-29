package frc.robot.hardware.motorcontrol

import com.batterystaple.kmeasure.*
import com.ctre.phoenix.motorcontrol.MotorCommutation
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.SensorInitializationStrategy
import frc.robot.hardware.interfaces.Encoder
import frc.robot.hardware.interfaces.EncoderMotorController
import frc.robot.hardware.interfaces.MotorConfigurable
import frc.robot.hardware.interfaces.MotorConfiguration
import frc.robot.hardware.interfaces.adaptors.CTREMotorControllerEncoderAdapter
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration as CTRETalonFXConfiguration

private const val TALON_FX_ENCODER_UNITS_PER_ROTATION = 2048 // From https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution

public fun falcon(canId: Int, canBus: String? = null): ChargerTalonFX =
    when {
        canBus != null -> ChargerTalonFX(canId, canBus)
        else -> ChargerTalonFX(canId)
    }

/**
 * Represents a TalonFX motor controller.
 * Includes everything in the CTRE TalonFX class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
 * @see TalonFXConfiguration
 */
public open class ChargerTalonFX(deviceNumber: Int, canBus: String = "rio") : WPI_TalonFX(deviceNumber, canBus),
    EncoderMotorController, MotorConfigurable<TalonFXConfiguration> {
    @Suppress("LeakingThis") // Known to be safe; CTREMotorControllerEncoderAdapter ONLY uses final functions
                                     // and does not pass around the reference to this class.
    final override val encoder: Encoder =
        CTREMotorControllerEncoderAdapter(
            ctreMotorController = this,
            pidIndex = 0, // Default
            pulsesPerRotation = TALON_FX_ENCODER_UNITS_PER_ROTATION
        )

    override fun configure(configuration: TalonFXConfiguration) {
        configuration.neutralMode?.let(::setNeutralMode)
        configuration.inverted?.let(::setInverted)
        configuration.expiration?.let { this.expiration = it.inUnit(Seconds) }
        configuration.safetyEnabled?.let(::setSafetyEnabled)
        configAllSettings(configuration.toCTRETalonFXConfiguration())
    }

    final override fun getSelectedSensorPosition(pidIdx: Int): Double =
        super.getSelectedSensorPosition(pidIdx)

    final override fun getSelectedSensorVelocity(pidIdx: Int): Double =
        super.getSelectedSensorVelocity(pidIdx)
}

/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonFX.
 *
 * @see ChargerTalonFX
 */
public data class TalonFXConfiguration(
    var neutralMode: NeutralMode? = null,
    var inverted: Boolean? = null,
    var expiration: Time? = null,
    var safetyEnabled: Boolean? = null,
    var supplyCurrentLimit: SupplyCurrentLimitConfiguration? = null,
    var statorCurrentLimit: StatorCurrentLimitConfiguration? = null,
    var motorCommutation: MotorCommutation? = null,
    var absoluteSensorRange: AbsoluteSensorRange? = null,
    var integratedSensorOffset: Angle? = null,
    var sensorInitializationStrategy: SensorInitializationStrategy? = null,
) : MotorConfiguration {
    public fun toCTRETalonFXConfiguration(): CTRETalonFXConfiguration =
        CTRETalonFXConfiguration()
            .also { ctreConfiguration ->
                supplyCurrentLimit?.let { ctreConfiguration.supplyCurrLimit = it }
                statorCurrentLimit?.let { ctreConfiguration.statorCurrLimit = it }
                motorCommutation?.let { ctreConfiguration.motorCommutation = it }
                absoluteSensorRange?.let { ctreConfiguration.absoluteSensorRange = it }
                integratedSensorOffset?.let { ctreConfiguration.integratedSensorOffsetDegrees = it.inUnit(Degrees) }
                sensorInitializationStrategy?.let { ctreConfiguration.initializationStrategy = sensorInitializationStrategy }
            }
}
