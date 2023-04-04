package frc.chargers.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.BaseTalon
import com.ctre.phoenix.sensors.CANCoder
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.controls.pid.PIDConstants
import kotlin.math.roundToInt
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced as CTREMotorController

public interface CTREMotorControllerConfiguration : MotorConfiguration {
    public var invertSensorPhase: Boolean?
    public var openLoopRampTimeFromNeutralToFull: Time?
    public var closedLoopRampTimeFromNeutralToFull: Time?
    public var peakOutputForwardPercent: Double?
    public var peakOutputReversePercent: Double?
    public var nominalOutputForwardPercent: Double?
    public var nominalOutputReversePercent: Double?
    public var neutralDeadbandPercent: Double?
    public var voltageCompensationSaturationVoltage: Voltage?
    public var voltageMeasurementFilterSamples: Int?
    public var voltageCompensationEnabled: Boolean?
    public val selectedFeedbackSensors: MutableMap<PIDIndex, FeedbackDevice>
    public val selectedFeedbackCoefficients: MutableMap<PIDIndex, Double>
    public var remoteFeedbackFilter: RemoteFeedbackFilterDevice?
    public val sensorTermFeedbackDevices: MutableMap<SensorTerm, FeedbackDevice>
    public val controlFramePeriods: MutableMap<ControlFrame, Time>
    public val statusFramePeriods: MutableMap<StatusFrame, Time>
    public var forwardLimitSwitchSource: LimitSwitchConfig?
    public var reverseLimitSwitchSource: LimitSwitchConfig?
    public var forwardSoftLimitThreshold: Angle?
    public var reverseSoftLimitThreshold: Angle?
    public var forwardSoftLimitEnable: Boolean?
    public var reverseSoftLimitEnable: Boolean?
    public val pidConfiguration: PIDConfiguration
    public val motionMagicConfiguration: MotionMagicConfiguration
    public val customParameters: MutableMap<CustomParameterIndex, CustomParameterValue>
    
    
    public sealed interface RemoteFeedbackFilterDevice
    public data class CANCoderRemoteFeedbackFilterDevice(val canCoder: CANCoder, val remoteOrdinal: Int) :
        RemoteFeedbackFilterDevice
    public data class BaseTalonRemoteFeedbackFilterDevice(val talon: BaseTalon, val remoteOrdinal: Int) :
        RemoteFeedbackFilterDevice
    public data class OtherRemoteFeedbackFilterDevice(val deviceId: Int, val remoteSensorSource: RemoteSensorSource, val remoteOrdinal: Int) :
        RemoteFeedbackFilterDevice
    public data class LimitSwitchConfig(val type: RemoteLimitSwitchSource, val normalOpenOrClose: LimitSwitchNormal, val deviceId: Int)
    public data class PIDConfiguration(
        val pidConstants: MutableMap<PIDIndex, PIDConstants> = mutableMapOf(),
        val feedForwards: MutableMap<PIDIndex, Double> = mutableMapOf(),
        val integralZones: MutableMap<PIDIndex, Double> = mutableMapOf(),
        val allowableClosedLoopErrors: MutableMap<PIDIndex, Double> = mutableMapOf(),
        val maxIntegralAccumulators: MutableMap<PIDIndex, Double> = mutableMapOf(),
        val closedLoopPeakOutputPercents: MutableMap<PIDIndex, Double> = mutableMapOf(),
        val closedLoopPeriods: MutableMap<PIDIndex, Time> = mutableMapOf(),
        var invertAuxPID: Boolean? = null,
        val profileSlots: MutableMap<SlotIndex, PIDIndex>? = null
    )
    public data class MotionMagicConfiguration(
        var cruiseVelocity: AngularVelocity? = null,
        var acceleration: AngularAcceleration? = null,
        var sCurveStrength: Int? = null,
        var baseTrajectoryPeriod: Time? = null,
    )
}

public var CTREMotorControllerConfiguration.selectedFeedbackSensor: FeedbackDevice?
    get() = selectedFeedbackSensors.getOrDefault(0, null)
    set(value) { value?.let { selectedFeedbackSensors[0] = it } ?: selectedFeedbackSensors.remove(0) }

internal fun CTREMotorController.configure(configuration: CTREMotorControllerConfiguration, encoderStep: Angle) {
    configuration.openLoopRampTimeFromNeutralToFull?.let { configOpenloopRamp(it.inUnit(seconds), TIMEOUT_MILLIS) }
    configuration.closedLoopRampTimeFromNeutralToFull?.let { configClosedloopRamp(it.inUnit(seconds), TIMEOUT_MILLIS) }
    configuration.peakOutputForwardPercent?.let { configPeakOutputForward(it, TIMEOUT_MILLIS) }
    configuration.peakOutputReversePercent?.let { configPeakOutputReverse(it, TIMEOUT_MILLIS) }
    configuration.nominalOutputForwardPercent?.let { configNominalOutputForward(it, TIMEOUT_MILLIS) }
    configuration.nominalOutputReversePercent?.let { configNominalOutputReverse(it, TIMEOUT_MILLIS) }
    configuration.neutralDeadbandPercent?.let { configNeutralDeadband(it, TIMEOUT_MILLIS) }
    configuration.voltageCompensationSaturationVoltage?.let {
        configVoltageCompSaturation(it.inUnit(volts), TIMEOUT_MILLIS)
    }
    configuration.voltageMeasurementFilterSamples?.let {
        configVoltageMeasurementFilter(it, TIMEOUT_MILLIS)
    }
    configuration.voltageCompensationEnabled?.let(::enableVoltageCompensation)
    configuration.selectedFeedbackSensors.forEach { (pidIndex, feedbackDevice) ->
        configSelectedFeedbackSensor(feedbackDevice, pidIndex, TIMEOUT_MILLIS)
    }
    configuration.selectedFeedbackCoefficients.forEach { (pidIndex, coefficient) ->
        configSelectedFeedbackCoefficient(coefficient, pidIndex, TIMEOUT_MILLIS)
    }
    configuration.remoteFeedbackFilter?.let {
        when(it) {
            is CTREMotorControllerConfiguration.BaseTalonRemoteFeedbackFilterDevice -> configRemoteFeedbackFilter(it.talon, it.remoteOrdinal,
                TIMEOUT_MILLIS
            )
            is CTREMotorControllerConfiguration.CANCoderRemoteFeedbackFilterDevice -> configRemoteFeedbackFilter(it.canCoder, it.remoteOrdinal,
                TIMEOUT_MILLIS
            )
            is CTREMotorControllerConfiguration.OtherRemoteFeedbackFilterDevice -> configRemoteFeedbackFilter(it.deviceId, it.remoteSensorSource, it.remoteOrdinal,
                TIMEOUT_MILLIS
            )
        }
    }
    configuration.sensorTermFeedbackDevices.forEach { (sensorTerm, feedbackDevice) ->
        configSensorTerm(sensorTerm, feedbackDevice, TIMEOUT_MILLIS)
    }
    configuration.controlFramePeriods.forEach { (controlFrame, period) ->
        setControlFramePeriod(controlFrame, period.inUnit(milli.seconds).roundToInt())
    }
    configuration.statusFramePeriods.forEach { (statusFrame, period) ->
        setStatusFramePeriod(statusFrame, period.inUnit(milli.seconds).roundToInt(), TIMEOUT_MILLIS)
    }
    configuration.forwardLimitSwitchSource?.let {  (type, normalOpenOrClose, deviceId) ->
        configForwardLimitSwitchSource(type, normalOpenOrClose, deviceId, TIMEOUT_MILLIS)
    }
    configuration.reverseLimitSwitchSource?.let {  (type, normalOpenOrClose, deviceId) ->
        configReverseLimitSwitchSource(type, normalOpenOrClose, deviceId, TIMEOUT_MILLIS)
    }
    configuration.forwardSoftLimitThreshold?.let { configForwardSoftLimitThreshold(it.inUnit(encoderStep),
        TIMEOUT_MILLIS
    ) }
    configuration.reverseSoftLimitThreshold?.let { configReverseSoftLimitThreshold(it.inUnit(encoderStep),
        TIMEOUT_MILLIS
    ) }
    configuration.forwardSoftLimitEnable?.let { configForwardSoftLimitEnable(it, TIMEOUT_MILLIS) }
    configuration.reverseSoftLimitEnable?.let { configReverseSoftLimitEnable(it, TIMEOUT_MILLIS) }
    configuration.pidConfiguration.let { (pidConstants, feedForwards, integralZones, allowableClosedLoopErrors,
                                             maxIntegralAccumulators, closedLoopPeakOutputPercents,
                                             closedLoopPeriods, invertAuxPID, profileSlots) ->
        pidConstants.forEach { i, (kP, kI, kD) ->
            config_kP(i, kP, TIMEOUT_MILLIS)
            config_kI(i, kI, TIMEOUT_MILLIS)
            config_kD(i, kD, TIMEOUT_MILLIS)
        }
        feedForwards.forEach { (i, kF) -> config_kF(i, kF, TIMEOUT_MILLIS) }
        integralZones.forEach { (i, integralZone) -> config_IntegralZone(i, integralZone, TIMEOUT_MILLIS) }
        allowableClosedLoopErrors.forEach { (i, allowableClosedLoopError) -> configAllowableClosedloopError(i, allowableClosedLoopError,
            TIMEOUT_MILLIS
        ) }
        maxIntegralAccumulators.forEach { (i, maxIntegralAccumulator) -> configMaxIntegralAccumulator(i, maxIntegralAccumulator,
            TIMEOUT_MILLIS
        ) }
        closedLoopPeakOutputPercents.forEach { (i, closedLoopPeakOutput) -> configClosedLoopPeakOutput(i, closedLoopPeakOutput,
            TIMEOUT_MILLIS
        ) }
        closedLoopPeriods.forEach { (i, closedLoopPeriod) -> configClosedLoopPeriod(i, closedLoopPeriod.inUnit(milli.seconds).roundToInt(),
            TIMEOUT_MILLIS
        ) }
        invertAuxPID?.let { configAuxPIDPolarity(it, TIMEOUT_MILLIS) }
        profileSlots?.forEach { (slotIndex, pidIndex) -> selectProfileSlot(slotIndex, pidIndex) }
    }
    configuration.motionMagicConfiguration.let { (cruiseVelocity, acceleration, sCurveStrength, baseTrajectoryPeriod) ->
        cruiseVelocity?.let { configMotionCruiseVelocity(cruiseVelocity.inUnit(encoderStep / 100.milli.seconds),
            TIMEOUT_MILLIS
        ) }
        acceleration?.let { configMotionAcceleration(acceleration.inUnit(encoderStep / 100.milli.seconds / seconds),
            TIMEOUT_MILLIS
        ) }
        sCurveStrength?.let { configMotionSCurveStrength(sCurveStrength, TIMEOUT_MILLIS) }
        baseTrajectoryPeriod?.let { configMotionProfileTrajectoryPeriod(it.inUnit(milli.seconds).roundToInt(),
            TIMEOUT_MILLIS
        ) }
    }
    configuration.customParameters.forEach { (i, customParameter) ->
        configSetCustomParam(customParameter, i, TIMEOUT_MILLIS)
    }
}

private const val TIMEOUT_MILLIS = 1000