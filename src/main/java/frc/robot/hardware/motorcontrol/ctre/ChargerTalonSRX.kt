package frc.robot.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.*
import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.SensorTerm
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import frc.robot.hardware.motorcontrol.EncoderMotorController
import frc.robot.hardware.motorcontrol.MotorConfigurable
import frc.robot.hardware.motorcontrol.ctre.CTREMotorControllerConfiguration.*
import frc.robot.hardware.sensors.encoders.CTREMotorControllerEncoderAdapter
import frc.robot.hardware.sensors.encoders.Encoder

private const val TALON_SRX_ENCODER_UNITS_PER_ROTATION = 2048 // From https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution

/**
 * Represents a TalonSRX motor controller.
 * Includes everything in the CTRE TalonSRX class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 * @see TalonSRXConfiguration
 */
public open class ChargerTalonSRX(
    deviceNumber: Int,
    protected val encoderTicksPerRotation: Int
) : WPI_TalonSRX(deviceNumber), EncoderMotorController, MotorConfigurable<TalonSRXConfiguration> {
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

        configure(configuration, encoderStep = (1.0 / TALON_SRX_ENCODER_UNITS_PER_ROTATION).ofUnit(Rotations))
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
    override var invertSensorPhase: Boolean? = null,
    var expiration: Time? = null,
    var safetyEnabled: Boolean? = null,
    override var openLoopRampSecondsFromNeutralToFull: Double? = null,
    override var closedLoopRampSecondsFromNeutralToFull: Double? = null,
    override var peakOutputForwardPercent: Double? = null,
    override var peakOutputReversePercent: Double? = null,
    override var nominalOutputForwardPercent: Double? = null,
    override var nominalOutputReversePercent: Double? = null,
    override var neutralDeadbandPercent: Double? = null,
    override var voltageCompensationSaturationVoltage: Double? = null,
    override var voltageMeasurementFilterSamples: Int? = null,
    override var voltageCompensationEnabled: Boolean? = null,
    override val selectedFeedbackSensors: MutableMap<PIDIndex, FeedbackDevice> = mutableMapOf(),
    override val selectedFeedbackCoefficients: MutableMap<PIDIndex, Double> = mutableMapOf(),
    override var remoteFeedbackFilter: RemoteFeedbackFilterDevice? = null,
    override val sensorTermFeedbackDevices: MutableMap<SensorTerm, FeedbackDevice> = mutableMapOf(),
    override val controlFramePeriods: MutableMap<ControlFrame, Time> = mutableMapOf(),
    override val statusFramePeriods: MutableMap<StatusFrame, Time> = mutableMapOf(),
    override var forwardLimitSwitchSource: LimitSwitchConfig? = null,
    override var reverseLimitSwitchSource: LimitSwitchConfig? = null,
    override var forwardSoftLimitThreshold: Angle? = null,
    override var reverseSoftLimitThreshold: Angle? = null,
    override var forwardSoftLimitEnable: Boolean? = null,
    override var reverseSoftLimitEnable: Boolean? = null,
    override val pidConfiguration: PIDConfiguration = PIDConfiguration(),
    override val motionMagicConfiguration: MotionMagicConfiguration = MotionMagicConfiguration(),
    override val customParameters: MutableMap<CustomParameterIndex, CustomParameterValue> = mutableMapOf()
) : CTREMotorControllerConfiguration