package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.*
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax.SoftLimitDirection
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.delay
import kotlin.math.roundToInt


public sealed class SparkMaxEncoderType{
    /**
     * Represents a regular spark max encoder.
     */
    public data object Regular: SparkMaxEncoderType()

    /**
     * Represents a Spark Max Alternate encoder.
     */
    public data class Alternate(
        val countsPerRev: Int,
        val encoderMeasurementPeriod: Time? = null,
        val type: SparkMaxAlternateEncoder.Type = SparkMaxAlternateEncoder.Type.kQuadrature
    ): SparkMaxEncoderType()

    /**
     * Represents an absolute encoder connected to a spark max.
     */
    public data class Absolute(
        val type: SparkMaxAbsoluteEncoder.Type = SparkMaxAbsoluteEncoder.Type.kDutyCycle
    ): SparkMaxEncoderType()
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
    factoryDefault: Boolean = true,
    encoderType: SparkMaxEncoderType = SparkMaxEncoderType.Regular,
    configure: SparkMaxConfiguration.() -> Unit = {}
): ChargerCANSparkMax =
    ChargerCANSparkMax(canBusId, CANSparkMaxLowLevel.MotorType.kBrushless, encoderType)
        .also {
            if (factoryDefault) {
                it.restoreFactoryDefaults()
                delay(200.milli.seconds)
                println("SparkMax has been factory defaulted.")
            }
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
    factoryDefault: Boolean = true,
    encoderType: SparkMaxEncoderType = SparkMaxEncoderType.Regular,
    configure: SparkMaxConfiguration.() -> Unit = {}
): ChargerCANSparkMax =
    ChargerCANSparkMax(canBusId, CANSparkMaxLowLevel.MotorType.kBrushed, encoderType)
        .also {
            if (factoryDefault) {
                it.restoreFactoryDefaults()
                delay(200.milli.seconds)
                println("SparkMax has been factory defaulted.")
            }
            it.configure(SparkMaxConfiguration().apply(configure))
        }



public class SparkMaxEncoderAdaptor(
    private val revEncoder: MotorFeedbackSensor
) : ResettableEncoder{

    init{
        require (
            revEncoder is AbsoluteEncoder || revEncoder is RelativeEncoder
        ){"Encoder type of spark max is invalid: internal error."}
    }

    private fun getPosition(): Angle = when (revEncoder){
        is AbsoluteEncoder -> revEncoder.position.ofUnit(rotations)
        is RelativeEncoder -> revEncoder.position.ofUnit(rotations)
        else -> error("Invalid encoder type for spark max")
    }

    private fun getVelocity(): AngularVelocity = when (revEncoder){
        is AbsoluteEncoder -> revEncoder.velocity.ofUnit(rotations / seconds)
        is RelativeEncoder -> revEncoder.velocity.ofUnit(rotations / minutes)
        else -> error("Invalid encoder type for spark max")
    }

    internal fun setInverted(value: Boolean): REVLibError = when (revEncoder){
        is AbsoluteEncoder -> revEncoder.setInverted(value)
        is RelativeEncoder -> revEncoder.setInverted(value)
        else -> error("Invalid encoder type for spark max")
    }

    internal fun setAverageDepth(depth: Int): REVLibError = when(revEncoder){
        is AbsoluteEncoder -> revEncoder.setAverageDepth(depth)
        is RelativeEncoder -> revEncoder.setAverageDepth(depth)
        else -> error("Invalid encoder type for spark max")
    }


    private var previousPosition = getPosition()
    private var previousVelocity = AngularVelocity(0.0)

    override fun setZero(newZero: Angle){
        when (revEncoder){
            is AbsoluteEncoder -> revEncoder.setZeroOffset(newZero.inUnit(rotations))
            is RelativeEncoder -> revEncoder.setPosition(newZero.inUnit(rotations))
        }
    }
    override val angularPosition: Angle
        get() = getPosition()
            .revertIfInvalid(previousPosition)
            .also{ previousPosition = it }

    override val angularVelocity: AngularVelocity
        get() = getVelocity()
            .revertIfInvalid(previousVelocity)
            .also{ previousVelocity = it }
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
    encoderType: SparkMaxEncoderType = SparkMaxEncoderType.Regular
) : CANSparkMax(deviceId, type), SmartEncoderMotorController, HardwareConfigurable<SparkMaxConfiguration>{
    final override val encoder: SparkMaxEncoderAdaptor = when (encoderType){
        SparkMaxEncoderType.Regular -> SparkMaxEncoderAdaptor(
            super.getEncoder()
        )

        is SparkMaxEncoderType.Alternate -> SparkMaxEncoderAdaptor(
            super.getAlternateEncoder(encoderType.type, encoderType.countsPerRev).also{
                if (encoderType.encoderMeasurementPeriod != null){
                    it.measurementPeriod = encoderType.encoderMeasurementPeriod.inUnit(milli.seconds).toInt()
                }
            }
        )

        is SparkMaxEncoderType.Absolute -> SparkMaxEncoderAdaptor(
            super.getAbsoluteEncoder(encoderType.type)
        )
    }



    private var previousCurrent = Current(0.0)
    private var previousTemp = 0.0
    private var previousVoltage = Voltage(0.0)

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
    private var currentPIDConstants = PIDConstants(0.0,0.0,0.0)
    private var currentFFConstants = AngularMotorFFConstants.None
    private var currentFF = Feedforward<AngularVelocityDimension, VoltageDimension>{ Voltage(0.0) }

    private fun updateControllerConstants(newConstants: PIDConstants){
        if (currentPIDConstants != newConstants){
            innerController.setP(newConstants.kP,0)
            innerController.setI(newConstants.kI,0)
            innerController.setD(newConstants.kD,0)
            currentPIDConstants = newConstants
        }
    }

    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
    ) {
        updateControllerConstants(pidConstants)
        if (currentFFConstants != feedforwardConstants){
            currentFFConstants = feedforwardConstants
            currentFF = Feedforward(currentFFConstants)
        }
        innerController.setReference(
            target.siValue,
            ControlType.kVelocity,
            0,
            currentFF.calculate(target).inUnit(volts)
        )
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


    private val allConfigErrors: LinkedHashSet<REVLibError> = linkedSetOf()
    private var configAppliedProperly = true
    private fun REVLibError.updateConfigStatus(): REVLibError{
        if (this != REVLibError.kOk){
            allConfigErrors.add(this)
            configAppliedProperly = false
        }
        return this
    }




    override fun configure(configuration: SparkMaxConfiguration) {
        configAppliedProperly = true
        // chargerlib defined function used for safe configuration.
        safeConfigure(
            deviceName = "ChargerCANSparkMax(id = $deviceId)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ){
            // ?.let only calls the function(with it as the receiver)
            // if the configuration is not null.
            // thus, it also returns a RevlibError, which will be processed
            // by the motor to determine if all configurations have gone through or not.
            allConfigErrors.clear()
            configuration.idleMode?.let(::setIdleMode)?.updateConfigStatus()
            configuration.inverted?.let(::setInverted)
            configuration.voltageCompensationNominalVoltage?.let { enableVoltageCompensation(it.inUnit(volts)) }?.updateConfigStatus()
            configuration.canTimeout?.let { timeout -> setCANTimeout(timeout.inUnit(milli.seconds).roundToInt()) }?.updateConfigStatus()
            configuration.closedLoopRampRate?.let(::setClosedLoopRampRate)?.updateConfigStatus()
            configuration.openLoopRampRate?.let(::setOpenLoopRampRate)?.updateConfigStatus()
            configuration.controlFramePeriod?.let { period -> setControlFramePeriodMs(period.inUnit(milli.seconds).roundToInt()) }
            for ((frame, period) in configuration.periodicFramePeriods) {
                setPeriodicFramePeriod(frame, period.inUnit(milli.seconds).roundToInt())?.updateConfigStatus()
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
            }?.updateConfigStatus()
            configuration.secondaryCurrentLimit?.let { (limit, chopCycles) ->
                when {
                    chopCycles != null -> setSecondaryCurrentLimit(limit.inUnit(amperes), chopCycles)
                    else -> setSecondaryCurrentLimit(limit.inUnit(amperes))
                }
            }?.updateConfigStatus()
            for ((limitDirection, limit) in configuration.softLimits) {
                setSoftLimit(limitDirection, limit.inUnit(rotations).toFloat())?.updateConfigStatus()
            }
            configuration.encoderAverageDepth?.let{ encoder.setAverageDepth(it) }?.updateConfigStatus()
            configuration.encoderInverted?.let{ encoder.setInverted(it) }?.updateConfigStatus()

            return@safeConfigure configAppliedProperly
        }

        if (RobotBase.isReal()) delay(200.milli.seconds)
        burnFlash()
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

    var encoderAverageDepth: Int? = null,
    var encoderInverted: Boolean? = null,

    // sparkMaxPIDController configs; not used by ChargerLib
    var feedbackDFilter: Double? = null,
    var feedbackIZone: Double? = null,
    var feedbackOutputRange: ClosedRange<Double>? = null,
    var positionPIDWrappingEnabled: Boolean? = null,
    var positionPIDWrappingInputRange: ClosedRange<Double>? = null,
) : HardwareConfiguration

public data class SmartCurrentLimit(val stallLimit: Current, val freeLimit: Current? = null, val limitSpeed: AngularVelocity? = null)
public data class SecondaryCurrentLimit(val limit: Current, val chopCycles: Int? = null)