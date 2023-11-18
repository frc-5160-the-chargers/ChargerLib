package frc.chargers.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.warnIfInSimulation
import frc.chargers.hardware.motorcontrol.FeedbackMotorController
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.relative.TalonFXEncoderAdapter
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import com.ctre.phoenix6.configs.TalonFXConfiguration as CTRETalonFXConfiguration


/**
 * Creates an instance of a falcon motor, controlled by a TalonFX motor controller.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup.
 * This setting can be changed by setting factoryDefault = false.
 */
public fun falcon(
    canId: Int,
    canBus: String? = null,
    factoryDefault: Boolean = true
): ChargerTalonFX = when {
    canBus != null -> ChargerTalonFX(canId, canBus)
    else -> ChargerTalonFX(canId)
}.apply {
    // factory defaults configs on startup is factoryDefault = true
    if (factoryDefault) {
        configurator.apply(CTRETalonFXConfiguration(), 0.02)
    }
}


/**
 * Creates an instance of a falcon motor, controlled by a TalonFX motor controller.
 * This function supports inline configuration using the configure lambda function,
 * which has the context of a [TalonFXConfiguration].
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 */
public inline fun falcon(
    canId: Int,
    canBus: String? = null,
    factoryDefault: Boolean = true,
    configure: TalonFXConfiguration.() -> Unit
): ChargerTalonFX =
    falcon(canId,canBus,factoryDefault).apply {
        // configures the motor by "applying" a a context function(configure)
        // onto a configuration object, then configuring using that config object.
        configure(TalonFXConfiguration().apply(configure))
    }


/**
 * Represents a TalonFX motor controller.
 * Includes everything in the CTRE TalonFX class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see com.ctre.phoenix6.hardware.TalonFX
 * @see TalonFXConfiguration
 */
public open class ChargerTalonFX(deviceNumber: Int, canBus: String = "rio") : TalonFX(deviceNumber, canBus),
    FeedbackMotorController, MotorConfigurable<TalonFXConfiguration> {

    init{
        warnIfInSimulation("ChargerTalonFX(ID = $deviceID)")
    }

    @Suppress("LeakingThis") // Known to be safe; CTREMotorControllerEncoderAdapter ONLY uses final functions
    // and does not pass around the reference to this class.
    final override val encoder: TalonFXEncoderAdapter =
        TalonFXEncoderAdapter(this)

    final override fun configure(configuration: TalonFXConfiguration){
        val baseTalonFXConfig = CTRETalonFXConfiguration()
        configurator.refresh(baseTalonFXConfig)
        baseTalonFXConfig.applyChanges(configuration)
        configurator.apply(baseTalonFXConfig,0.050)


        configuration.apply{
            positionUpdateFrequency?.let{
                position.setUpdateFrequency(it.inUnit(hertz))
            }

            velocityUpdateFrequency?.let{
                velocity.setUpdateFrequency(it.inUnit(hertz))
            }

            motorOutputUpdateFrequency?.let{
                supplyVoltage.setUpdateFrequency(it.inUnit(hertz))
                dutyCycle.setUpdateFrequency(it.inUnit(hertz))
            }

            currentUpdateFrequency?.let{
                torqueCurrent.setUpdateFrequency(it.inUnit(hertz))
                supplyCurrent.setUpdateFrequency(it.inUnit(hertz))
                statorCurrent.setUpdateFrequency(it.inUnit(hertz))
            }
        }
        println("TalonFX has been configured.")

    }


    /*
    Below is the Implementation of the FeedbackMotorController interface.
     */

    private val currentMMConfigs = MotionMagicConfigs()
    private val currentSlotConfigs = Slot0Configs()
    private val velocityRequest = VelocityVoltage(0.0).also{ it.Slot = 0 }
    private val positionRequest = PositionVoltage(0.0).also{it.Slot = 0 }
    private val mmRequest = MotionMagicVoltage(0.0).also{it.Slot = 0 }

    final override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF
    ) {
        if (currentSlotConfigs.pidConstants != pidConstants ||
            currentSlotConfigs.kS != feedforward.kS.inUnit(volts) ||
            currentSlotConfigs.kV != feedforward.getKV(rotations,seconds)){

            currentSlotConfigs.pidConstants = pidConstants
            currentSlotConfigs.kS = feedforward.kS.inUnit(volts)
            currentSlotConfigs.kV = feedforward.getKV(rotations,seconds)
            configurator.apply(currentSlotConfigs)
        }

        velocityRequest.Velocity = target.inUnit(rotations/seconds)
        velocityRequest.FeedForward = (feedforward.getAccelerationVoltage() + feedforward.gravity.get()).inUnit(volts)
        setControl(velocityRequest)
    }

    final override fun setAngularPosition(target: Angle, pidConstants: PIDConstants, absoluteEncoder: PositionEncoder?) {

        if (currentSlotConfigs.pidConstants != pidConstants){
            currentSlotConfigs.pidConstants = pidConstants
            configurator.apply(currentSlotConfigs)
        }

        /**
         * Onboard PID control usually does not support external Encoders,
         * especially not ones that aren't from the motor vendor.
         *
         * This essentially uses a workaround to allow an external Encoder to be used.
         *
         * Let's assume that the integrated Encoder returns a position of 540 degrees,
         * the [absoluteEncoder] returns a position of 180 degrees,
         * and the target is 90 degrees.
         *
         * By setting the target of the PID loop to 540 - (180-90) = 450 degrees,
         * the same effect is acheived, since the motor is spinning 90 degrees to the left in both instances.
         */
        if (absoluteEncoder == null){
            positionRequest.Position = target.inUnit(rotations)
        }else{
            positionRequest.Position = (encoder.angularPosition - (absoluteEncoder.angularPosition - target)).inUnit(rotations)
        }
        setControl(positionRequest)
    }

    final override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF,
        constraints: AngularTrapezoidProfile.Constraints,
        absoluteEncoder: PositionEncoder?
    ) {
        if(constraints.maxVelocity != currentMMConfigs.MotionMagicCruiseVelocity.ofUnit(rotations/seconds)
            || constraints.maxAcceleration != currentMMConfigs.MotionMagicAcceleration.ofUnit(rotations/seconds/seconds)){
            currentMMConfigs.MotionMagicCruiseVelocity = constraints.maxVelocity.inUnit(rotations/seconds)
            currentMMConfigs.MotionMagicAcceleration = constraints.maxAcceleration.inUnit(rotations/seconds/seconds)
            configurator.apply(currentMMConfigs)
        }

        if (currentSlotConfigs.pidConstants != pidConstants ||
            currentSlotConfigs.kS != feedforward.kS.inUnit(volts) ||
            currentSlotConfigs.kV != feedforward.getKV(rotations,seconds)){

            currentSlotConfigs.pidConstants = pidConstants
            currentSlotConfigs.kS = feedforward.kS.inUnit(volts)
            currentSlotConfigs.kV = feedforward.getKV(rotations,seconds)
            configurator.apply(currentSlotConfigs)
        }

        /**
         * See above for an explanation.
         */
        if (absoluteEncoder == null){
            mmRequest.Position = target.inUnit(rotations)
        }else{
            mmRequest.Position = (encoder.angularPosition - (absoluteEncoder.angularPosition - target)).inUnit(rotations)
        }
        mmRequest.FeedForward = (feedforward.getAccelerationVoltage() + feedforward.gravity.get()).inUnit(volts)
        setControl(mmRequest)
    }


}






/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonFX.
 *
 * Identical to CTRE's TalonFXConfiguration for Phoenix v6, except for a couple of changes:
 *
 * 1. This configuration is designed to "apply" changes onto the existing configuration instead of overriding them.
 *    Thus, a configuration with the value null actually means a configuration that is not modified
 *    compared to the current one.
 *
 *
 * 2. PID / Motion magic configuration is removed, and replaced with FeedbackMotorController functionality.
 *
 * @see ChargerTalonFX
 */
public data class TalonFXConfiguration(


    // audio configs
    var beepOnBoot: Boolean? = null,

    // closed loop general configs

    // Note: While this value is by default false in CTRE's configuration,
    // We want it to be true.
    var closedLoopContinuousWrap: Boolean? = null,

    // Closed Loop Ramps Configs
    var dutyCycleClosedLoopRampPeriod: Time? = null,
    var torqueClosedLoopRampPeriod: Time? = null,
    var voltageClosedLoopRampPeriod: Time? = null,

    // Open loop ramp configs
    var dutyCycleOpenLoopRampPeriod: Time? = null,
    var torqueOpenLoopRampPeriod: Time? = null,
    var voltageOpenLoopRampPeriod: Time? = null,

    // Current Limit Configs
    var statorCurrentLimitEnable: Boolean? = null,
    var supplyCurrentLimitEnable: Boolean? = null,
    var statorCurrentLimit: Current? = null,
    var supplyCurrentLimit: Current? = null,
    var supplyCurrentThreshold: Current? = null,
    var supplyTimeThreshold: Time? = null,

    // feedback configs
    var feedbackRemoteSensorID: Int? = null,
    var feedbackRotorOffset: Angle? = null,
    var feedbackSensorSource: FeedbackSensorSourceValue? = null,
    var rotorToSensorRatio: Double? = null,
    var sensorToMechanismRatio: Double? = null,

    // Hardware Limit Switch Configs
    var forwardLimitEnable: Boolean? = null,
    var forwardLimitAutosetPositionEnable: Boolean? = null,
    var forwardLimitAutosetPositionValue: Angle? = null,
    var forwardLimitRemoteSensorID: Int? = null,
    var forwardLimitSource: ForwardLimitSourceValue? = null,
    var forwardLimitType: ForwardLimitTypeValue? = null,

    var reverseLimitEnable: Boolean? = null,
    var reverseLimitAutosetPositionEnable: Boolean? = null,
    var reverseLimitAutosetPositionValue: Angle? = null,
    var reverseLimitRemoteSensorID: Int? = null,
    var reverseLimitSource: ReverseLimitSourceValue? = null,
    var reverseLimitType: ReverseLimitTypeValue? = null,

    // Motor Output Configs

    var neutralMode: NeutralModeValue? = null,
    var inverted: Boolean? = null,
    var dutyCycleNeutralDeadband: Double? = null,
    var peakForwardDutyCycle: Double? = null,
    var peakReverseDutyCycle: Double? = null,

    // Software Limit Switch Configs

    var forwardSoftLimitEnable: Boolean? = null,
    var reverseSoftLimitEnable: Boolean? = null,
    var forwardSoftLimitThreshold: Angle? = null,
    var reverseSoftLimitThreshold: Angle? = null,

    // Torque Current Configs
    var peakForwardTorqueCurrent: Current? = null,
    var peakReverseTorqueCurrent: Current? = null,
    var torqueNeutralDeadband: Current? = null,

    // Voltage Configs

    var peakForwardVoltage: Voltage? = null,
    var peakReverseVoltage: Voltage? = null,
    var supplyVoltageTimeConstant: Time? = null,

    var positionUpdateFrequency: Frequency? = null,
    var velocityUpdateFrequency: Frequency? = null,
    var motorOutputUpdateFrequency: Frequency? = null,
    var currentUpdateFrequency: Frequency? = null

): MotorConfiguration

public fun CTRETalonFXConfiguration.applyChanges(chargerConfig: TalonFXConfiguration): CTRETalonFXConfiguration{
    chargerConfig.beepOnBoot?.let{
        Audio.BeepOnBoot = it
    }

    chargerConfig.closedLoopContinuousWrap?.let{
        ClosedLoopGeneral.ContinuousWrap = it
    }

    CurrentLimits.apply{
        chargerConfig.statorCurrentLimitEnable?.let{ StatorCurrentLimitEnable = it }
        chargerConfig.statorCurrentLimit?.let{ StatorCurrentLimit = it.inUnit(amps) }
        chargerConfig.supplyCurrentLimit?.let{ SupplyCurrentLimit = it.inUnit(amps) }
        chargerConfig.supplyCurrentLimitEnable?.let{ SupplyCurrentLimitEnable = it }
        chargerConfig.supplyCurrentThreshold?.let{ SupplyCurrentThreshold = it.inUnit(amps) }
        chargerConfig.supplyTimeThreshold?.let{ SupplyTimeThreshold = it.inUnit(seconds) }
    }

    ClosedLoopRamps.apply{
        chargerConfig.dutyCycleClosedLoopRampPeriod?.let{ DutyCycleClosedLoopRampPeriod = it.inUnit(seconds) }
        chargerConfig.torqueClosedLoopRampPeriod?.let{ TorqueClosedLoopRampPeriod = it.inUnit(seconds) }
        chargerConfig.voltageClosedLoopRampPeriod?.let{ VoltageClosedLoopRampPeriod = it.inUnit(seconds) }
    }
    OpenLoopRamps.apply{
        chargerConfig.dutyCycleOpenLoopRampPeriod?.let{ DutyCycleOpenLoopRampPeriod = it.inUnit(seconds) }
        chargerConfig.torqueOpenLoopRampPeriod?.let{ TorqueOpenLoopRampPeriod = it.inUnit(seconds) }
        chargerConfig.voltageOpenLoopRampPeriod?.let{ VoltageOpenLoopRampPeriod = it.inUnit(seconds) }
    }

    Feedback.apply{
        chargerConfig.feedbackRemoteSensorID?.let{ FeedbackRemoteSensorID = it }
        chargerConfig.feedbackRotorOffset?.let{ FeedbackRotorOffset = it.inUnit(rotations) }
        chargerConfig.feedbackSensorSource?.let{ FeedbackSensorSource = it }
        chargerConfig.rotorToSensorRatio?.let{ RotorToSensorRatio = it }
        chargerConfig.sensorToMechanismRatio?.let{ SensorToMechanismRatio = it }
    }
    
    HardwareLimitSwitch.apply{
        chargerConfig.forwardLimitEnable?.let{ ForwardLimitEnable = it }
        chargerConfig.forwardLimitAutosetPositionEnable?.let{ ForwardLimitAutosetPositionEnable = it }
        chargerConfig.forwardLimitAutosetPositionValue?.let{ ForwardLimitAutosetPositionValue = it.inUnit(rotations) }
        chargerConfig.forwardLimitRemoteSensorID?.let{ForwardLimitRemoteSensorID = it}
        chargerConfig.forwardLimitSource?.let{ ForwardLimitSource = it }
        chargerConfig.forwardLimitType?.let{ ForwardLimitType = it }


        chargerConfig.reverseLimitEnable?.let{ ReverseLimitEnable = it }
        chargerConfig.reverseLimitAutosetPositionEnable?.let{ ReverseLimitAutosetPositionEnable = it }
        chargerConfig.reverseLimitAutosetPositionValue?.let{ ReverseLimitAutosetPositionValue = it.inUnit(rotations) }
        chargerConfig.reverseLimitRemoteSensorID?.let{ReverseLimitRemoteSensorID = it}
        chargerConfig.reverseLimitSource?.let{ ReverseLimitSource = it }
        chargerConfig.reverseLimitType?.let{ ReverseLimitType = it }
    }

    MotorOutput.apply{
        chargerConfig.inverted?.let{ Inverted = if (it) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive }
        chargerConfig.neutralMode?.let{ NeutralMode = it }
        chargerConfig.dutyCycleNeutralDeadband?.let{ DutyCycleNeutralDeadband = it}
        chargerConfig.peakForwardDutyCycle?.let{PeakForwardDutyCycle = it}
        chargerConfig.peakReverseDutyCycle?.let{ PeakReverseDutyCycle = it }
    }

    SoftwareLimitSwitch.apply{
        chargerConfig.forwardSoftLimitEnable?.let{ ForwardSoftLimitEnable = it }
        chargerConfig.forwardSoftLimitThreshold?.let{ ForwardSoftLimitThreshold = it.inUnit(rotations) }
        chargerConfig.reverseSoftLimitEnable?.let{ ReverseSoftLimitEnable = it }
        chargerConfig.reverseSoftLimitThreshold?.let{ ReverseSoftLimitThreshold = it.inUnit(rotations) }
    }

    TorqueCurrent.apply{
        chargerConfig.peakForwardTorqueCurrent?.let{ PeakForwardTorqueCurrent = it.inUnit(amps) }
        chargerConfig.peakReverseTorqueCurrent?.let{ PeakReverseTorqueCurrent = it.inUnit(amps) }
        chargerConfig.torqueNeutralDeadband?.let{ TorqueNeutralDeadband = it.inUnit(amps) }
    }

    Voltage.apply{
        chargerConfig.peakForwardVoltage?.let{ PeakForwardVoltage = it.inUnit(volts) }
        chargerConfig.peakReverseVoltage?.let{ PeakReverseVoltage = it.inUnit(volts) }
        chargerConfig.supplyVoltageTimeConstant?.let{ SupplyVoltageTimeConstant = it.inUnit(seconds) }
    }




    return this
}



public var Slot0Configs.pidConstants: PIDConstants
    get() = PIDConstants(kP,kI,kD)
    set(newConstants){
        kP = newConstants.kP
        kI = newConstants.kI
        kD = newConstants.kD
    }




