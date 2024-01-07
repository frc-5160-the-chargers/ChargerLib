package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.*
import com.revrobotics.CANSparkLowLevel.*
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.motorcontrol.rev.util.*
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.delay


/**
 * A convenience function to create a [ChargerSparkMax]
 * specifically to drive a Neo motor.
 *
 * This function supports inline configuration using the "[configure]" lambda function,
 * which has the context of a [SparkMaxConfiguration] object.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 *
 * ```
 * // example
 * val neo = neoSparkMax(deviceId = 5){ inverted = false }
 */
public inline fun neoSparkMax(
    canBusId: Int,
    factoryDefault: Boolean = true,
    configure: SparkMaxConfiguration.() -> Unit = {}
): ChargerSparkMax =
    ChargerSparkMax(canBusId, MotorType.kBrushless)
        .apply {
            if (factoryDefault) {
                restoreFactoryDefaults()
                delay(200.milli.seconds)
                println("SparkMax has been factory defaulted.")
            }
            val config = SparkMaxConfiguration().apply(configure)
            configure(config)
        }


/**
 * A convenience function to create a [ChargerSparkMax]
 * specifically to drive a brushed motor, such as a CIM.
 *
 * This motor supports inline configuration using the "[configure]" lambda function,
 * which has the context of a [SparkMaxConfiguration] object.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 *
 * ```
 * // example
 * val neo = neoSparkMax(deviceId = 5){ inverted = false }
 */
public inline fun brushedSparkMax(
    canBusId: Int,
    factoryDefault: Boolean = true,
    configure: SparkMaxConfiguration.() -> Unit = {}
): ChargerSparkMax =
    ChargerSparkMax(canBusId, MotorType.kBrushed)
        .apply {
            if (factoryDefault) {
                restoreFactoryDefaults()
                delay(200.milli.seconds)
                println("SparkMax has been factory defaulted.")
            }
            val config = SparkMaxConfiguration().apply(configure)
            configure(config)
        }

/**
 * Represents a type of spark max encoder.
 *
 * Options include [Regular], [Alternate] or [Absolute].
 */
public sealed class SparkMaxEncoderType{
    /**
     * Represents a regular spark max encoder.
     */
    public data class Regular(
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkMaxEncoderType()

    /**
     * Represents a Spark Max Alternate encoder.
     */
    public data class Alternate(
        val type: SparkMaxAlternateEncoder.Type,
        val countsPerRev: Int,
        val encoderMeasurementPeriod: Time? = null,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkMaxEncoderType()

    /**
     * Represents an absolute encoder connected to a spark max.
     */
    public data class Absolute(
        val type: SparkAbsoluteEncoder.Type,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkMaxEncoderType()
}


/**
 * A class that represents possible configurations for a spark max motor controller.
 *
 * @see ChargerSparkMax
 */
public class SparkMaxConfiguration(
    public var encoderType: SparkMaxEncoderType? = null,
    idleMode: CANSparkBase.IdleMode? = null,
    inverted: Boolean? = null,
    voltageCompensationNominalVoltage: Voltage? = null,
    canTimeout: Time? = null,
    closedLoopRampRate: Double? = null,
    openLoopRampRate: Double? = null,
    controlFramePeriod: Time? = null,
    periodicFramePeriods: MutableMap<PeriodicFrame, Time> = mutableMapOf(),
    smartCurrentLimit: SmartCurrentLimit? = null,
    secondaryCurrentLimit: SecondaryCurrentLimit? = null,
    softLimits: MutableMap<CANSparkBase.SoftLimitDirection, Angle> = mutableMapOf(),
): SparkConfigurationBase(
    idleMode, inverted, voltageCompensationNominalVoltage, canTimeout, closedLoopRampRate, openLoopRampRate,
    controlFramePeriod, periodicFramePeriods, smartCurrentLimit, secondaryCurrentLimit, softLimits
)






/**
 * Represents a Spark Max motor controller.
 * Includes everything in the REV Robotics [CANSparkMax] class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see com.revrobotics.CANSparkMax
 * @see SparkMaxConfiguration
 */
public class ChargerSparkMax(
    deviceId: Int,
    type: MotorType
) : CANSparkMax(deviceId, type), SmartEncoderMotorController, HardwareConfigurable<SparkMaxConfiguration>{

    override var encoder: SparkEncoderAdaptor = getEncoder(SparkMaxEncoderType.Regular())
        private set

    private fun getEncoder(encoderType: SparkMaxEncoderType): SparkEncoderAdaptor =
        when (encoderType){
            is SparkMaxEncoderType.Regular -> SparkEncoderAdaptor(
                super.getEncoder().apply{
                    // property access syntax setters
                    if (encoderType.averageDepth != null){
                        averageDepth = encoderType.averageDepth
                    }
                    if (encoderType.inverted != null){
                        inverted = encoderType.inverted
                    }
                }
            )

            is SparkMaxEncoderType.Alternate -> SparkEncoderAdaptor(
                super.getAlternateEncoder(
                    SparkMaxAlternateEncoder.Type.kQuadrature,
                    encoderType.countsPerRev
                ).apply{
                    // property access syntax setters
                    if (encoderType.encoderMeasurementPeriod != null){
                        measurementPeriod = encoderType.encoderMeasurementPeriod.inUnit(milli.seconds).toInt()
                    }
                    if (encoderType.averageDepth != null){
                        averageDepth = encoderType.averageDepth
                    }
                    if (encoderType.inverted != null){
                        inverted = encoderType.inverted
                    }
                }
            )

            is SparkMaxEncoderType.Absolute -> SparkEncoderAdaptor(
                super.getAbsoluteEncoder(encoderType.type).apply{
                    // property access syntax setters
                    if (encoderType.averageDepth != null){
                        averageDepth = encoderType.averageDepth
                    }
                    if (encoderType.inverted != null){
                        inverted = encoderType.inverted
                    }
                }
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



    private val pidHandler = SparkPIDHandler(motor = this)

    override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean,
        extraVoltage: Voltage
    ): Unit = pidHandler.setAngularPosition(target, pidConstants, continuousWrap, extraVoltage)

    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
    ): Unit = pidHandler.setAngularVelocity(target, pidConstants, feedforwardConstants)




    private var allConfigErrors: List<REVLibError> = listOf()

    override fun configure(configuration: SparkMaxConfiguration) {
        configuration.encoderType?.let{ encoderType ->
            encoder = getEncoder(encoderType)
        }
        // chargerlib defined function used for safe configuration.
        safeConfigure(
            deviceName = "ChargerSparkMax(id = $deviceId)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ){
            // configures the motor and records errors
            allConfigErrors = configuration.applyTo(this)
            return@safeConfigure allConfigErrors.isEmpty()
        }
        if (RobotBase.isReal()) delay(200.milli.seconds)
        burnFlash()
    }

}
