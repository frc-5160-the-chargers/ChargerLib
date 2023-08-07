package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax.SoftLimitDirection
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame
import com.revrobotics.SparkMaxAlternateEncoder
import com.revrobotics.SparkMaxPIDController
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.FeedbackMotorController
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.hardware.sensors.encoders.RevEncoderAdapter
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import kotlin.math.PI
import kotlin.math.roundToInt

/**
 * A convenience function to create a [ChargerCANSparkMax]
 * specifically to drive a Neo motor.
 */
public inline fun neoSparkMax(canBusId: Int, alternateEncoderConfiguration: AlternateEncoderConfiguration? = null, configure: SparkMaxConfiguration.() -> Unit = {}): ChargerCANSparkMax =
    ChargerCANSparkMax(canBusId, CANSparkMaxLowLevel.MotorType.kBrushless, alternateEncoderConfiguration)
        .also { it.configure(SparkMaxConfiguration().apply(configure)) }

/**
 * A convenience function to create a [ChargerCANSparkMax]
 * specifically to drive a brushed motor, such as a CIM.
 */
public inline fun brushedSparkMax(canBusId: Int, alternateEncoderConfiguration: AlternateEncoderConfiguration? = null, configure: SparkMaxConfiguration.() -> Unit = {}): ChargerCANSparkMax =
    ChargerCANSparkMax(canBusId, CANSparkMaxLowLevel.MotorType.kBrushed, alternateEncoderConfiguration)
        .also { it.configure(SparkMaxConfiguration().apply(configure)) }

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
) : CANSparkMax(deviceId, type), FeedbackMotorController, MotorConfigurable<SparkMaxConfiguration> {


    override val encoder: ResettableEncoder by lazy {
        alternateEncoderConfiguration?.let { (countsPerRev, encoderType) ->
            if (encoderType == null) {
                RevEncoderAdapter(super.getAlternateEncoder(countsPerRev))
            } else {
                RevEncoderAdapter(super.getAlternateEncoder(encoderType, countsPerRev))
            }
        }
            ?: RevEncoderAdapter(super.getEncoder())
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
        burnFlash()
    }


    /*
    Below is the implementation of the FeedbackMotorController Interface
     */



    // equivalent to SparkMax.getPIDController() (uses property access syntax)
    private val innerController = pidController.also{
        it.positionPIDWrappingEnabled = true
        // SI units are used by default here, so radians are used.
        it.positionPIDWrappingMaxInput = PI
        it.positionPIDWrappingMinInput = -PI

    }
    private var currentConstants = PIDConstants(0.0,0.0,0.0)

    private fun updateControllerConstants(newConstants: PIDConstants){
        if(currentConstants != newConstants){
            innerController.constants = newConstants
            currentConstants = newConstants
        }
    }

    private var SparkMaxPIDController.constants: PIDConstants
        get() = PIDConstants(getP(0), getI(0), getD(0) )
        set(newConstants){
            setP(newConstants.kP,0)
            setI(newConstants.kI,0)
            setD(newConstants.kD,0)
        }
    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF
    ) {
        updateControllerConstants(pidConstants)
        innerController.setReference(target.siValue, ControlType.kVelocity,0,feedforward.calculate(target).inUnit(volts))
    }

    override fun setAngularPosition(target: Angle, pidConstants: PIDConstants,absoluteEncoder: Encoder?) {
        val actualTarget = if (absoluteEncoder != null){
            encoder.angularPosition - (absoluteEncoder.angularPosition - target)
        }else{
            target
        }
        updateControllerConstants(pidConstants)
        innerController.setReference(actualTarget.siValue,ControlType.kPosition,0)
    }

    override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF,
        constraints: AngularTrapezoidProfile.Constraints,
        absoluteEncoder: Encoder?
    ) {
        val trapezoidProfile = AngularTrapezoidProfile(
            constraints,
            AngularTrapezoidProfile.State(target,AngularVelocity(0.0)),
            AngularTrapezoidProfile.State(encoder.angularPosition,AngularVelocity(0.0))
        )
        val currentState = trapezoidProfile.calculateCurrentState()
        updateControllerConstants(pidConstants)
        if (absoluteEncoder != null){
            innerController.setReference((currentState.position + (encoder.angularPosition - absoluteEncoder.angularPosition)).siValue, ControlType.kPosition,0,feedforward.calculate(currentState.velocity).inUnit(volts))
        }else{
            innerController.setReference(currentState.position.siValue, ControlType.kPosition,0,feedforward.calculate(currentState.velocity).inUnit(volts))
        }
    }
}


/**
 * A data class representing all possible configuration parameters
 * of a ChargerCANSparkMax.
 *
 * @see ChargerCANSparkMax
 */
public data class SparkMaxConfiguration(
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
) : MotorConfiguration

public data class AlternateEncoderConfiguration(val countsPerRev: Int, val encoderType: SparkMaxAlternateEncoder.Type? = null) {
    public companion object {
        public val SRXMagnetic1X: AlternateEncoderConfiguration = AlternateEncoderConfiguration(1024)
        public val SRXMagnetic4X: AlternateEncoderConfiguration = AlternateEncoderConfiguration(4096)
    }
}
public data class SmartCurrentLimit(val stallLimit: Current, val freeLimit: Current? = null, val limitSpeed: AngularVelocity? = null)
public data class SecondaryCurrentLimit(val limit: Current, val chopCycles: Int? = null)