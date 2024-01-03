package frc.chargers.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import com.ctre.phoenix6.configs.TalonFXConfiguration as CTRETalonFXConfiguration

/**
 * Creates an instance of a falcon motor, controlled by a TalonFX motor controller.
 * This function supports inline configuration using the configure lambda function,
 * which has the context of a [TalonFXConfiguration].
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 */
public fun falcon(
    canId: Int,
    canBus: String = "rio",
    factoryDefault: Boolean = true,
    // This function block has the context of a TalonFXConfiguration,
    // which means that it is basically treated as if it was called with the class itself.
    // Example Usage: falcon(6){ beepOnBoot = true; neutralMode = NeutralModeValue.Brake }
    configure: TalonFXConfiguration.() -> Unit = {}
): ChargerTalonFX = ChargerTalonFX(canId, canBus).apply {
    // factory defaults configs on startup when factoryDefault = true
    if (factoryDefault) {
        configurator.apply(CTRETalonFXConfiguration(), 0.02)
    }
    val config = TalonFXConfiguration().apply(configure)
    if (config != TalonFXConfiguration()){
        configure(config)
    }
}



public class TalonFXEncoderAdapter(
    private val motorController: TalonFX
): ResettableEncoder {
    private val positionSignal = motorController.position
    private val velocitySignal = motorController.velocity

    override fun setZero(newZero: Angle) {
        val errorCode = motorController.setPosition(newZero.inUnit(rotations))
        if (errorCode != StatusCode.OK){
            error("When attempting to zero a talon fx, an error occurred: $errorCode")
        }
    }

    override val angularPosition: Angle
        get() = positionSignal.refresh(true).value.ofUnit(rotations)

    override val angularVelocity: AngularVelocity
        get() = velocitySignal.refresh(true).value.ofUnit(rotations/seconds)
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
public open class ChargerTalonFX(deviceNumber: Int, canBus: String = "rio"):
    TalonFX(deviceNumber, canBus), SmartEncoderMotorController, HardwareConfigurable<TalonFXConfiguration> {

    @Suppress("LeakingThis") // Known to be safe; CTREMotorControllerEncoderAdapter ONLY uses final functions
    // and does not pass around the reference to this class.
    /**
     * The encoder of the TalonFX.
     */
    final override val encoder: TalonFXEncoderAdapter =
        TalonFXEncoderAdapter(this)


    private val voltageSignal = supplyVoltage
    private val currentSignal = statorCurrent
    private val tempSignal = deviceTemp

    override val appliedVoltage: Voltage
        get() = voltageSignal.refresh(true).value.ofUnit(volts)

    override val appliedCurrent: Current
        get() = currentSignal.refresh(true).value.ofUnit(amps)

    override val tempCelsius: Double
        get() = tempSignal.refresh(true).value


    private val currentSlotConfigs = Slot0Configs()
    private val velocityRequest = VelocityVoltage(0.0).also{ it.Slot = 0 }
    private val positionRequest = PositionVoltage(0.0).also{it.Slot = 0 }

    private fun currentPIDConstants() = PIDConstants(currentSlotConfigs.kP, currentSlotConfigs.kI, currentSlotConfigs.kD)

    private fun setPIDConstants(newConstants: PIDConstants){
        currentSlotConfigs.kP = newConstants.kP
        currentSlotConfigs.kI = newConstants.kI
        currentSlotConfigs.kD = newConstants.kD
    }


    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
    ) {
        var configHasChanged = false
        if (currentPIDConstants() != pidConstants){
            setPIDConstants(pidConstants)
            configHasChanged = true
        }
        if (currentSlotConfigs.kS != feedforwardConstants.kS.inUnit(volts) || currentSlotConfigs.kV != feedforwardConstants.kV.inUnit(volts * seconds / rotations)){
            currentSlotConfigs.kS = feedforwardConstants.kS.inUnit(volts)
            currentSlotConfigs.kV = feedforwardConstants.kV.inUnit(volts * seconds / rotations)
            configHasChanged = true
        }
        if (configHasChanged){
            configurator.apply(currentSlotConfigs)
            println("PID status has been updated.")
        }
        velocityRequest.Velocity = target.inUnit(rotations/seconds)
        setControl(velocityRequest)
    }

    override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        absoluteEncoder: PositionEncoder?,
        extraVoltage: Voltage
    ) {
        if (currentPIDConstants() != pidConstants){
            setPIDConstants(pidConstants)
            configurator.apply(currentSlotConfigs)
            println("PID status has been updated.")
        }
        if (absoluteEncoder == null){
            positionRequest.Position = target.inUnit(rotations)
        }else{
            positionRequest.Position = (encoder.angularPosition - absoluteEncoder.angularPosition + target).inUnit(rotations)
        }
        positionRequest.FeedForward = extraVoltage.inUnit(volts)
        setControl(positionRequest)
    }



    private val allConfigErrors: LinkedHashSet<StatusCode> = linkedSetOf()
    private var configAppliedProperly = true
    private fun StatusCode.updateConfigStatus(): StatusCode {
        if (this != StatusCode.OK){
            allConfigErrors.add(this)
            configAppliedProperly = false
        }
        return this
    }


    final override fun configure(configuration: TalonFXConfiguration){
        configAppliedProperly = true
        safeConfigure(
            deviceName = "ChargerTalonFX(id = $deviceID)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ){
            allConfigErrors.clear()
            val baseTalonFXConfig = CTRETalonFXConfiguration()
            configurator.refresh(baseTalonFXConfig)
            applyChanges(baseTalonFXConfig, configuration)
            configurator.apply(baseTalonFXConfig,0.02).updateConfigStatus()

            configuration.apply{
                positionUpdateFrequency?.let{
                    position.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }

                velocityUpdateFrequency?.let{
                    velocity.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }

                motorOutputUpdateFrequency?.let{
                    supplyVoltage.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                    dutyCycle.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }

                currentUpdateFrequency?.let{
                    torqueCurrent.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                    supplyCurrent.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                    statorCurrent.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }
            }
            return@safeConfigure configAppliedProperly
        }
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
 *    compared to the current one. Since motors are factory-defaulted upon initialization using convenience functions,
 *    it is not nessecary to have overriding configuration.
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

): HardwareConfiguration

internal fun applyChanges(ctreConfig: CTRETalonFXConfiguration, chargerConfig: TalonFXConfiguration): CTRETalonFXConfiguration{
    ctreConfig.apply{
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
    }
    return ctreConfig
}






