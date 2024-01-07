package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.REVLibError
import frc.chargers.hardware.configuration.HardwareConfiguration
import kotlin.math.roundToInt


public data class SmartCurrentLimit(
    val stallLimit: Current,
    val freeLimit: Current? = null,
    val limitSpeed: AngularVelocity? = null
)

public data class SecondaryCurrentLimit(
    val limit: Current,
    val chopCycles: Int? = null
)


/**
 * A base class that represents configuration for REV motors;
 * these include the ChargerCANSparkMax and the ChargerCANSparkFlex.
 *
 * These configurations have a 1-to-1 correspondence with the "set" functions of [CANSparkBase];
 * however, Kmeasure is used whenever possible.
 */
@Suppress("MemberVisibilityCanBePrivate")
public abstract class SparkConfigurationBase(
    public var idleMode: CANSparkBase.IdleMode? = null,
    public var inverted: Boolean? = null,
    public var voltageCompensationNominalVoltage: Voltage? = null,
    public var canTimeout: Time? = null,
    public var closedLoopRampRate: Double? = null,
    public var openLoopRampRate: Double? = null,
    public var controlFramePeriod: Time? = null,
    public var periodicFramePeriods: MutableMap<CANSparkLowLevel.PeriodicFrame, Time> = mutableMapOf(),
    public var smartCurrentLimit: SmartCurrentLimit? = null,
    public var secondaryCurrentLimit: SecondaryCurrentLimit? = null,
    public var softLimits: MutableMap<CANSparkBase.SoftLimitDirection, Angle> = mutableMapOf(),
): HardwareConfiguration {

    public fun setPeriodicFramePeriod(periodicFrame: CANSparkLowLevel.PeriodicFrame, period: Time){
        periodicFramePeriods[periodicFrame] = period
    }

    public fun setSoftLimit(direction: CANSparkBase.SoftLimitDirection, limit: Angle){
        softLimits[direction] = limit
    }

    public fun setSmartCurrentLimit(limit: Current){
        smartCurrentLimit = SmartCurrentLimit(limit)
    }

    public fun setSmartCurrentLimit(
        stallLimit: Current,
        freeLimit: Current,
        limitSpeed: AngularVelocity? = null
    ){
        smartCurrentLimit = SmartCurrentLimit(stallLimit, freeLimit, limitSpeed)
    }

    public fun setSecondaryCurrentLimit(limit: Current, chopCycles: Int? = null){
        secondaryCurrentLimit = SecondaryCurrentLimit(limit, chopCycles)
    }


    internal fun applyTo(motor: CANSparkBase): List<REVLibError>{
        val allErrors: MutableList<REVLibError> = mutableListOf()

        fun REVLibError?.updateConfigStatus(): REVLibError? {
            if (this != null && this != REVLibError.kOk) {
                allErrors.add(this)
            }
            return this
        }

        // ?.let only calls the function(with it as the receiver)
        // if the configuration is not null.
        // thus, it also returns a RevlibError, which will be processed
        // by the motor to determine if all configurations have gone through or not.
        idleMode?.let(motor::setIdleMode).updateConfigStatus()
        inverted?.let(motor::setInverted)
        voltageCompensationNominalVoltage?.let { motor.enableVoltageCompensation(it.inUnit(volts)) }.updateConfigStatus()
        canTimeout?.let { timeout -> motor.setCANTimeout(timeout.inUnit(milli.seconds).roundToInt()) }.updateConfigStatus()
        closedLoopRampRate?.let(motor::setClosedLoopRampRate).updateConfigStatus()
        openLoopRampRate?.let(motor::setOpenLoopRampRate).updateConfigStatus()
        controlFramePeriod?.let { period -> motor.setControlFramePeriodMs(period.inUnit(milli.seconds).roundToInt()) }
        for ((frame, period) in periodicFramePeriods) {
            motor.setPeriodicFramePeriod(frame, period.inUnit(milli.seconds).roundToInt()).updateConfigStatus()
        }
        smartCurrentLimit?.let { (stallLimit, freeLimit, limitSpeed) ->
            when {
                limitSpeed != null && freeLimit != null ->
                    motor.setSmartCurrentLimit(
                        stallLimit.inUnit(amps).roundToInt(),
                        freeLimit.inUnit(amps).roundToInt(),
                        limitSpeed.inUnit(rotations / minutes).roundToInt()
                    )
                freeLimit != null -> motor.setSmartCurrentLimit(
                    stallLimit.inUnit(amps).roundToInt(),
                    freeLimit.inUnit(amps).roundToInt()
                )
                else -> motor.setSmartCurrentLimit(
                    stallLimit.inUnit(amps).roundToInt()
                )
            }
        }.updateConfigStatus()
        secondaryCurrentLimit?.let { (limit, chopCycles) ->
            when {
                chopCycles != null -> motor.setSecondaryCurrentLimit(limit.inUnit(amperes), chopCycles)
                else -> motor.setSecondaryCurrentLimit(limit.inUnit(amperes))
            }
        }.updateConfigStatus()
        for ((limitDirection, limit) in softLimits) {
            motor.setSoftLimit(limitDirection, limit.inUnit(rotations).toFloat()).updateConfigStatus()
        }
        return allErrors
    }
}