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
import frc.chargers.hardware.motorcontrol.FeedbackMotorController
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.sensors.encoders.TalonFXEncoderAdapter
import frc.chargers.hardware.sensors.encoders.TimestampedEncoder
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import com.ctre.phoenix6.configs.TalonFXConfiguration as CTRETalonFXConfiguration



// private const val TALON_FX_ENCODER_UNITS_PER_ROTATION = 2048 // From https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution

public inline fun falcon(canId: Int, canBus: String? = null, configure: TalonFXConfiguration.() -> Unit = {}): ChargerTalonFX =
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
 * @see com.ctre.phoenix6.hardware.TalonFX
 * @see TalonFXConfiguration
 */
public open class ChargerTalonFX(deviceNumber: Int, canBus: String = "rio") : TalonFX(deviceNumber, canBus),
    FeedbackMotorController, MotorConfigurable<TalonFXConfiguration> {

    @Suppress("LeakingThis") // Known to be safe; CTREMotorControllerEncoderAdapter ONLY uses final functions
    // and does not pass around the reference to this class.
    final override val encoder: TimestampedEncoder =
        TalonFXEncoderAdapter(this)

    final override fun configure(configuration: TalonFXConfiguration){
        configurator.apply(configuration.toCTRETalonFXConfiguration())
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
        velocity: AngularVelocity,
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

        velocityRequest.Velocity = velocity.inUnit(rotations/seconds)
        velocityRequest.FeedForward = (feedforward.getAccelerationVoltage() + feedforward.gravity.getOutput()).inUnit(volts)
        setControl(velocityRequest)
    }

    final override fun setAngularPosition(position: Angle, pidConstants: PIDConstants) {
        if (currentSlotConfigs.pidConstants != pidConstants){
            currentSlotConfigs.pidConstants = pidConstants
            configurator.apply(currentSlotConfigs)
        }
        positionRequest.Position = position.inUnit(rotations)
        setControl(positionRequest)
    }

    final override fun setAngularPosition(
        position: Angle,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF,
        constraints: AngularTrapezoidProfile.Constraints
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

        mmRequest.Position = position.inUnit(rotations)
        mmRequest.FeedForward = (feedforward.getAccelerationVoltage() + feedforward.gravity.getOutput()).inUnit(volts)
        setControl(mmRequest)
    }


}





/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonFX.
 *
 * Identical to CTRE's TalonFXConfiguration for Phoenix v6, except for a couple of changes:
 *
 * 1. All configs with an "Enable" counterpart now use kotlin's nullable types.
 *    For example, instead of having to set [StatorCurrentLimitEnable] to false,
 *    you just set [statorCurrentLimit] to null instead.
 *
 * 2. Configurations are no longer grouped into sub-configurations,
 *    as it isn't really necessary anymore with lambda-based configuration.
 *
 * 3. PID / Motion magic configuration is removed, and replaced with FeedbackMotorController functionality
 *
 * @see ChargerTalonFX
 */

// Note to self: CustomParamConfigs might be able to be used by chargerlib, no idea how
public data class TalonFXConfiguration(
    /*
    Important note: TalonFXConfiguration actually doesn't use null-by-default properties,
    like SparkMaxConfiguration and TalonSRXConfiguration.

    This is because the way CTRE motor controllers are configured, through phoenix v6,
    is through another TalonFXConfiguration object.
    Since changing the values of properties within an object is negligible performance-wise,
    nullable types can be used for the various "enable" properties within the TalonFXConfiguration instead.

     */


    // audio configs
    var beepOnBoot: Boolean = true,

    // closed loop general configs

    // Note: While this value is by default false in CTRE's configuration,
    // We want it to be true.
    var closedLoopContinuousWrap: Boolean = true,

    // Closed Loop Ramps Configs
    var dutyCycleClosedLoopRampPeriod: Time = Time(0.0),
    var torqueClosedLoopRampPeriod: Time = Time(0.0),
    var voltageClosedLoopRampPeriod: Time = Time(0.0),

    // Open loop ramp configs
    var dutyCycleOpenLoopRampPeriod: Time = Time(0.0),
    var torqueOpenLoopRampPeriod: Time = Time(0.0),
    var voltageOpenLoopRampPeriod: Time = Time(0.0),

    // Current Limit Configs
    var statorCurrentLimit: Current? = null,
    var supplyCurrentLimit: Current? = null,
    var supplyCurrentThreshold: Current = Current(0.0),
    var supplyTimeThreshold: Time = Time(0.0),

    // feedback configs
    var feedbackRemoteSensorID: Int = 0,
    var feedbackRotorOffset: Angle = Angle(0.0),
    var feedbackSensorSource: FeedbackSensorSourceValue = FeedbackSensorSourceValue.RotorSensor,
    var rotorToSensorRatio: Double = 1.0,
    var sensorToMechanismRatio: Double = 1.0,

    // Hardware Limit Switch Configs
    var forwardLimitEnable: Boolean = false,
    var forwardLimitAutosetPositionValue: Angle? = null,
    var forwardLimitRemoteSensorID: Int = 0,
    var forwardLimitSource: ForwardLimitSourceValue = ForwardLimitSourceValue.LimitSwitchPin,
    var forwardLimitType: ForwardLimitTypeValue = ForwardLimitTypeValue.NormallyOpen,

    var reverseLimitEnable: Boolean = false,
    var reverseLimitAutosetPositionValue: Angle? = null,
    var reverseLimitRemoteSensorID: Int = 0,
    var reverseLimitSource: ReverseLimitSourceValue = ReverseLimitSourceValue.LimitSwitchPin,
    var reverseLimitType: ReverseLimitTypeValue = ReverseLimitTypeValue.NormallyOpen,

    // Motor Output Configs

    var neutralMode: NeutralModeValue = NeutralModeValue.Brake,
    var inverted: Boolean = false,
    var dutyCycleNeutralDeadband: Double = 0.0,
    var peakForwardDutyCycle: Double = 1.0,
    var peakReverseDutyCycle: Double = -1.0,

    // Software Limit Switch Configs

    var forwardSoftLimitThreshold: Angle? = null,
    var reverseSoftLimitThreshold: Angle? = null,

    // Torque Current Configs
    var peakForwardTorqueCurrent: Current = 800.amps,
    var peakReverseTorqueCurrent: Current = -800.amps,
    var torqueNeutralDeadband: Current = Current(0.0),

    // Voltage Configs

    var peakForwardVoltage: Voltage = 12.volts,
    var peakReverseVoltage: Voltage = -12.volts,
    var supplyVoltageTimeConstant: Time = Time(0.0)

): MotorConfiguration{

    public fun toCTRETalonFXConfiguration(): CTRETalonFXConfiguration{
        val config = CTRETalonFXConfiguration()
        config.Audio.BeepOnBoot = beepOnBoot

        config.ClosedLoopGeneral.ContinuousWrap = closedLoopContinuousWrap

        config.ClosedLoopRamps.apply{
            DutyCycleClosedLoopRampPeriod = dutyCycleClosedLoopRampPeriod.inUnit(seconds)
            TorqueClosedLoopRampPeriod = torqueClosedLoopRampPeriod.inUnit(seconds)
            VoltageClosedLoopRampPeriod = voltageClosedLoopRampPeriod.inUnit(seconds)
        }

        config.CurrentLimits.apply{

            if(statorCurrentLimit == null){
                StatorCurrentLimitEnable = false
            }else{
                StatorCurrentLimitEnable = true
                // In this specific scenario(as well as others),
                // the "!!"(aka assert non-null call is perfectly safe.
                StatorCurrentLimit = statorCurrentLimit!!.inUnit(amps)
            }

            if(supplyCurrentLimit == null){
                SupplyCurrentLimitEnable = false
            }else{
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = supplyCurrentLimit!!.inUnit(amps)
            }
            SupplyCurrentThreshold = supplyCurrentThreshold.inUnit(amps)
            SupplyTimeThreshold = supplyTimeThreshold.inUnit(seconds)
        }

        config.OpenLoopRamps.apply{
            DutyCycleOpenLoopRampPeriod = dutyCycleOpenLoopRampPeriod.inUnit(seconds)
            TorqueOpenLoopRampPeriod = torqueOpenLoopRampPeriod.inUnit(seconds)
            VoltageOpenLoopRampPeriod = voltageOpenLoopRampPeriod.inUnit(seconds)
        }

        config.Feedback.apply{
            FeedbackRemoteSensorID = feedbackRemoteSensorID
            FeedbackRotorOffset = feedbackRotorOffset.inUnit(rotations)
            FeedbackSensorSource = feedbackSensorSource
            RotorToSensorRatio = rotorToSensorRatio
            SensorToMechanismRatio = sensorToMechanismRatio
        }

        config.HardwareLimitSwitch.apply{
            ForwardLimitEnable = forwardLimitEnable
            if(forwardLimitAutosetPositionValue == null){
                ForwardLimitAutosetPositionEnable = false
            }else{
                ForwardLimitAutosetPositionEnable = true
                ForwardLimitAutosetPositionValue = forwardLimitAutosetPositionValue!!.inUnit(rotations)
            }

            ForwardLimitRemoteSensorID = forwardLimitRemoteSensorID
            ForwardLimitSource = forwardLimitSource
            ForwardLimitType = forwardLimitType


            ReverseLimitEnable = reverseLimitEnable
            if(reverseLimitAutosetPositionValue == null){
                ReverseLimitAutosetPositionEnable = false
            }else{
                ReverseLimitAutosetPositionEnable = true
                ReverseLimitAutosetPositionValue = reverseLimitAutosetPositionValue!!.inUnit(rotations)
            }
            ReverseLimitRemoteSensorID = reverseLimitRemoteSensorID
            ReverseLimitSource = reverseLimitSource
            ReverseLimitType = reverseLimitType
        }


        config.MotorOutput.apply{
            Inverted = if(inverted){
                InvertedValue.Clockwise_Positive
            }else{
                InvertedValue.CounterClockwise_Positive
            }
            NeutralMode = neutralMode
            DutyCycleNeutralDeadband = dutyCycleNeutralDeadband
            PeakForwardDutyCycle = peakForwardDutyCycle
            PeakReverseDutyCycle = peakReverseDutyCycle
        }

        config.SoftwareLimitSwitch.apply{
            if(forwardSoftLimitThreshold == null){
                ForwardSoftLimitEnable = false
            }else{
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = forwardSoftLimitThreshold!!.inUnit(rotations)
            }

            if(reverseSoftLimitThreshold == null){
                ReverseSoftLimitEnable = false
            }else{
                ReverseSoftLimitEnable = true
                ReverseSoftLimitThreshold = reverseSoftLimitThreshold!!.inUnit(rotations)
            }
        }

        config.TorqueCurrent.apply{
            PeakForwardTorqueCurrent = peakForwardTorqueCurrent.inUnit(amps)
            PeakReverseTorqueCurrent = peakReverseTorqueCurrent.inUnit(amps)
            TorqueNeutralDeadband = torqueNeutralDeadband.inUnit(amps)
        }
        config.Voltage.apply{
            PeakForwardVoltage = peakForwardVoltage.inUnit(volts)
            PeakReverseVoltage = peakReverseVoltage.inUnit(volts)
            SupplyVoltageTimeConstant = supplyVoltageTimeConstant.inUnit(seconds)
        }

        return config
    }
}


public var Slot0Configs.pidConstants: PIDConstants
    get() = PIDConstants(kP,kI,kD)
    set(newConstants){
        kP = newConstants.kP
        kI = newConstants.kI
        kD = newConstants.kD
    }




