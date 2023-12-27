package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax.SoftLimitDirection
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame
import com.revrobotics.SparkMaxAlternateEncoder
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.relative.SparkMaxEncoderAdapter
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.delay
import kotlin.math.roundToInt


/**
 * A convenience function to create a [ChargerCANSparkMax]
 * specifically to drive a Neo motor.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup.
 * This setting can be changed by setting factoryDefault = false.
 */
public fun neoSparkMax(
    canBusId: Int,
    alternateEncoderConfiguration: AlternateEncoderConfiguration? = null,
    factoryDefault: Boolean = true,
): ChargerCANSparkMax =
    ChargerCANSparkMax(canBusId, CANSparkMaxLowLevel.MotorType.kBrushless, alternateEncoderConfiguration)
        .also {
            if (factoryDefault) {
                it.restoreFactoryDefaults()
                println("SparkMax has been factory defaulted.")
            }
        }


/**
 * A convenience function to create a [ChargerCANSparkMax]
 * specifically to drive a brushed motor, such as a CIM.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup.
 * This setting can be changed by setting factoryDefault = false.
 */
public fun brushedSparkMax(
    canBusId: Int,
    alternateEncoderConfiguration: AlternateEncoderConfiguration? = null,
    factoryDefault: Boolean = true,
): ChargerCANSparkMax =
    ChargerCANSparkMax(canBusId, CANSparkMaxLowLevel.MotorType.kBrushed, alternateEncoderConfiguration)
        .also {
            if (factoryDefault) {
                it.restoreFactoryDefaults()
                println("SparkMax has been factory defaulted.")
            }
        }



/**
 * A convenience function to create a [ChargerCANSparkMax]
 * specifically to drive a Neo motor.
 *
 * This motor supports inline configuration using the [configure] lambda function,
 * which has the context of a [SparkMaxConfiguration] object.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 */
public inline fun neoSparkMax(
    canBusId: Int,
    alternateEncoderConfiguration: AlternateEncoderConfiguration? = null,
    factoryDefault: Boolean = true,
    configure: SparkMaxConfiguration.() -> Unit
): ChargerCANSparkMax =
    neoSparkMax(canBusId, alternateEncoderConfiguration, factoryDefault)
        .also {
            it.configure(SparkMaxConfiguration().apply(configure))
        }

/**
 * A convenience function to create a [ChargerCANSparkMax]
 * specifically to drive a brushed motor, such as a CIM.
 *
 * This motor supports inline configuration using the [configure] lambda function,
 * which has the context of a [SparkMaxConfiguration] object.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 */
public inline fun brushedSparkMax(
    canBusId: Int,
    alternateEncoderConfiguration: AlternateEncoderConfiguration? = null,
    factoryDefault: Boolean = true,
    configure: SparkMaxConfiguration.() -> Unit
): ChargerCANSparkMax =
    brushedSparkMax(canBusId, alternateEncoderConfiguration, factoryDefault)
        .also {
            it.configure(SparkMaxConfiguration().apply(configure))
        }

/**
 * Represents a Spark Max motor controller.
 * Includes everything in the REV Robotics [CANSparkMax] class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see com.revrobotics.CANSparkMax
 * @see SparkMaxConfiguration
 */
public open class ChargerCANSparkMax(
    deviceId: Int,
    type: MotorType,
    alternateEncoderConfiguration: AlternateEncoderConfiguration? = null
) : CANSparkMax(deviceId, type), SmartEncoderMotorController, HardwareConfigurable<SparkMaxConfiguration>{


    private inner class EncoderConfiguration(
        var averageDepth: Int? = null,
        var inverted: Boolean? = null,
        var measurementPeriod: Time? = null,
        var positionConversionFactor: Double? = null,
        var velocityConversionFactor: Double? = null,
    )

    private var encoderConfig = EncoderConfiguration()

    private var previousCurrent = Current(0.0)
    private var previousTemp = 0.0
    private var previousVoltage = Voltage(0.0)


    /*
    CANSparkMax throws some exception if this is initialized on motor creation,
    so by lazy here delays the initialization until the encoder is accessed.
     */
    override val encoder: SparkMaxEncoderAdapter by lazy {
        (alternateEncoderConfiguration?.let { (countsPerRev, encoderType) ->
            if (encoderType == null) {
                SparkMaxEncoderAdapter(super.getAlternateEncoder(countsPerRev))
            } else {
                SparkMaxEncoderAdapter(super.getAlternateEncoder(encoderType, countsPerRev))
            }
        }
            ?: SparkMaxEncoderAdapter(super.getEncoder())
        ).apply{
            encoderConfig.averageDepth?.let{averageDepth = it}
            encoderConfig.inverted?.let{inverted = it}
            encoderConfig.measurementPeriod?.let{measurementPeriod = (it.inUnit(seconds) * 1000).toInt()}
            encoderConfig.positionConversionFactor?.let{positionConversionFactor = it}
            encoderConfig.velocityConversionFactor?.let{velocityConversionFactor = it}
        }
    }

    override val appliedCurrent: Current
        get() = outputCurrent.ofUnit(amps)
            .revertIfInvalid(previousCurrent)
            .also{ previousCurrent = it }

    override val tempCelsius: Double
        get() = motorTemperature
            .revertIfInvalid(previousTemp)
            .also{ previousTemp = it }

    override val appliedVoltage: Voltage
        get() = busVoltage.ofUnit(volts)
            .revertIfInvalid(previousVoltage)
            .also{ previousVoltage = it }


    // equivalent to SparkMax.getPIDController() (uses property access syntax)
    private val innerController = pidController
    private var currentConstants = PIDConstants(0.0,0.0,0.0)

    private fun updateControllerConstants(newConstants: PIDConstants){
        if (currentConstants != newConstants){
            innerController.setP(newConstants.kP,0)
            innerController.setI(newConstants.kI,0)
            innerController.setD(newConstants.kD,0)
            currentConstants = newConstants
        }
    }

    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF
    ) {
        updateControllerConstants(pidConstants)
        innerController.setReference(target.siValue, ControlType.kVelocity,0,feedforward.calculate(target).inUnit(volts))
    }

    public override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        absoluteEncoder: PositionEncoder?,
        extraVoltage: Voltage
    ) {
        val actualTarget = if (absoluteEncoder != null){
            encoder.angularPosition + (absoluteEncoder.angularPosition - target)
        }else{
            target
        }
        updateControllerConstants(pidConstants)
        innerController.setReference(actualTarget.siValue, ControlType.kPosition, 0, extraVoltage.siValue)
    }


    override fun configure(configuration: SparkMaxConfiguration) {
        configuration.idleMode?.let(::setIdleMode)
        configuration.inverted?.let(::setInverted)
        configuration.voltageCompensationNominalVoltage?.let { enableVoltageCompensation(it.inUnit(volts)) }
        configuration.canTimeout?.let { timeout -> setCANTimeout(timeout.inUnit(milli.seconds).roundToInt()) }
        configuration.closedLoopRampRate?.let(::setClosedLoopRampRate)
        configuration.openLoopRampRate?.let(::setOpenLoopRampRate)
        configuration.controlFramePeriod?.let { period -> setControlFramePeriodMs(period.inUnit(milli.seconds).roundToInt()) }
        for ((frame, period) in configuration.periodicFramePeriods) {
            setPeriodicFramePeriod(frame, period.inUnit(milli.seconds).roundToInt())
        }
        configuration.smartCurrentLimit?.let { (stallLimit, freeLimit, limitSpeed) ->
            when {
                limitSpeed != null && freeLimit != null ->
                    setSmartCurrentLimit(
                        stallLimit.inUnit(amps).roundToInt(),
                        freeLimit.inUnit(amps).roundToInt(),
                        limitSpeed.inUnit(rotations/minutes).roundToInt()
                    )
                freeLimit != null -> setSmartCurrentLimit(
                    stallLimit.inUnit(amps).roundToInt(),
                    freeLimit.inUnit(amps).roundToInt()
                )
                else -> setSmartCurrentLimit(
                    stallLimit.inUnit(amps).roundToInt()
                )
            }
        }
        configuration.secondaryCurrentLimit?.let { (limit, chopCycles) ->
            when {
                chopCycles != null -> setSecondaryCurrentLimit(limit.inUnit(amperes), chopCycles)
                else -> setSecondaryCurrentLimit(limit.inUnit(amperes))
            }
        }
        for ((limitDirection, limit) in configuration.softLimits) {
            setSoftLimit(limitDirection, limit.inUnit(rotations).toFloat())
        }



        if (RobotBase.isReal()) delay(200.milli.seconds)
        burnFlash()
        if (RobotBase.isReal()) delay(200.milli.seconds)

        println("SparkMax has been configured.")

    }


}


/**
 * A data class representing all possible configuration parameters
 * of a ChargerCANSparkMax.
 *
 * All properties here are variables and are nullable, with null representing no configuration at all.
 *
 * @see ChargerCANSparkMax
 */
public data class SparkMaxConfiguration(
    // motor configs
    var idleMode: IdleMode? = null,
    var inverted: Boolean? = null,
    var voltageCompensationNominalVoltage: Voltage? = null,
    var canTimeout: Time? = null,
    var closedLoopRampRate: Double? = null,
    var openLoopRampRate: Double? = null,
    var controlFramePeriod: Time? = null,
    val periodicFramePeriods: MutableMap<PeriodicFrame, Time> = mutableMapOf(),
    var smartCurrentLimit: SmartCurrentLimit? = null,
    var secondaryCurrentLimit: SecondaryCurrentLimit? = null,
    val softLimits: MutableMap<SoftLimitDirection, Angle> = mutableMapOf(),

    // encoder configs
    var encoderAverageDepth: Int? = null,
    var encoderInverted: Boolean? = null,
    var encoderMeasurementPeriod: Time? = null,
    var encoderPositionConversionFactor: Double? = null,
    var encoderVelocityConversionFactor: Double? = null,

    // sparkMaxPIDController configs; not used by ChargerLib
    var feedbackDFilter: Double? = null,
    var feedbackIZone: Double? = null,
    var feedbackOutputRange: ClosedRange<Double>? = null,
    var positionPIDWrappingEnabled: Boolean? = null,
    var positionPIDWrappingInputRange: ClosedRange<Double>? = null,
) : HardwareConfiguration

public data class AlternateEncoderConfiguration(val countsPerRev: Int, val encoderType: SparkMaxAlternateEncoder.Type? = null) {
    public companion object {
        public val SRXMagnetic1X: AlternateEncoderConfiguration = AlternateEncoderConfiguration(1024)
        public val SRXMagnetic4X: AlternateEncoderConfiguration = AlternateEncoderConfiguration(4096)
    }
}
public data class SmartCurrentLimit(val stallLimit: Current, val freeLimit: Current? = null, val limitSpeed: AngularVelocity? = null)
public data class SecondaryCurrentLimit(val limit: Current, val chopCycles: Int? = null)