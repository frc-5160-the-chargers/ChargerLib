package frc.robot.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.*
import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.SensorInitializationStrategy
import frc.robot.hardware.motorcontrol.EncoderMotorController
import frc.robot.hardware.motorcontrol.MotorConfigurable
import frc.robot.hardware.motorcontrol.ctre.CTREMotorControllerConfiguration.*
import frc.robot.hardware.sensors.encoders.CTREMotorControllerEncoderAdapter
import frc.robot.hardware.sensors.encoders.Encoder
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration as CTRETalonFXConfiguration

private const val TALON_FX_ENCODER_UNITS_PER_ROTATION = 2048 // From https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution

public fun falcon(canId: Int, canBus: String? = null, configure: TalonFXConfiguration.() -> Unit = {}): ChargerTalonFX =
    when {
        canBus != null -> ChargerTalonFX(canId, canBus)
        else -> ChargerTalonFX(canId)
    }.apply {
        configure(TalonFXConfiguration().apply(configure))
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
        configuration.invertSensorPhase?.let(::setSensorPhase)
        configuration.expiration?.let { this.expiration = it.inUnit(Seconds) }
        configuration.safetyEnabled?.let(::setSafetyEnabled)

        configure(configuration, encoderStep = (1.0 / TALON_FX_ENCODER_UNITS_PER_ROTATION).ofUnit(Rotations))

        configAllSettings(configuration.toCTRETalonFXConfiguration())
    }

    final override fun getSelectedSensorPosition(pidIdx: Int): Double =
        super.getSelectedSensorPosition(pidIdx)

    final override fun getSelectedSensorVelocity(pidIdx: Int): Double =
        super.getSelectedSensorVelocity(pidIdx)
}

public typealias PIDIndex = Int
public typealias SlotIndex = Int
public typealias CustomParameterIndex = Int
public typealias CustomParameterValue = Int

/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonFX.
 *
 * @see ChargerTalonFX
 */
public data class TalonFXConfiguration(
    var neutralMode: NeutralMode? = null,
    var inverted: Boolean? = null,
    override var invertSensorPhase: Boolean? = null,
    var expiration: Time? = null,
    var safetyEnabled: Boolean? = null,
    var supplyCurrentLimit: SupplyCurrentLimitConfiguration? = null,
    var statorCurrentLimit: StatorCurrentLimitConfiguration? = null,
    var motorCommutation: MotorCommutation? = null,
    var absoluteSensorRange: AbsoluteSensorRange? = null,
    var integratedSensorOffset: Angle? = null,
    var sensorInitializationStrategy: SensorInitializationStrategy? = null,
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
) : CTREMotorControllerConfiguration {
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
