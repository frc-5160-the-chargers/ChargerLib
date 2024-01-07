package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.revrobotics.*
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.rev.util.*
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.delay

/**
 * A convenience function to create a [ChargerSparkFlex].
 *
 * This function supports inline configuration using the "[configure]" lambda function,
 * which has the context of a [SparkMaxConfiguration] object.
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 *
 * Example:
 * ```
 * val neo = neoSparkFlex(deviceId = 5){ inverted = false }
 */
public inline fun neoSparkFlex(
    deviceId: Int,
    factoryDefault: Boolean = true,
    configure: SparkFlexConfiguration.() -> Unit = {}
): ChargerSparkFlex = ChargerSparkFlex(deviceId)
    .apply {
        if (factoryDefault) {
            restoreFactoryDefaults()
            delay(200.milli.seconds)
            println("SparkFlex has been factory defaulted.")
        }
        val config = SparkFlexConfiguration().apply(configure)
        configure(config)
    }



/**
 * Represents a type of spark flex encoder.
 *
 * Options include [Regular], [External] or [Absolute].
 */
public sealed class SparkFlexEncoderType{
    /**
     * Represents a regular Spark Flex encoder.
     */
    public data class Regular(
        val category: SparkRelativeEncoder.Type = SparkRelativeEncoder.Type.kHallSensor,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkFlexEncoderType()

    /**
     * Represents a Spark Flex Alternate encoder.
     */
    public data class External(
        val countsPerRev: Int,
        val category: SparkFlexExternalEncoder.Type = SparkFlexExternalEncoder.Type.kQuadrature,
        val encoderMeasurementPeriod: Time? = null,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkFlexEncoderType()

    /**
     * Represents an absolute encoder connected to a Spark Flex.
     */
    public data class Absolute(
        val category: SparkAbsoluteEncoder.Type,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkFlexEncoderType()
}


/**
 * A class that represents possible configurations for a spark max motor controller.
 *
 * @see ChargerSparkMax
 */
public class SparkFlexConfiguration(
    public var encoderType: SparkFlexEncoderType? = null,
    idleMode: CANSparkBase.IdleMode? = null,
    inverted: Boolean? = null,
    voltageCompensationNominalVoltage: Voltage? = null,
    canTimeout: Time? = null,
    closedLoopRampRate: Double? = null,
    openLoopRampRate: Double? = null,
    controlFramePeriod: Time? = null,
    periodicFramePeriods: MutableMap<CANSparkLowLevel.PeriodicFrame, Time> = mutableMapOf(),
    smartCurrentLimit: SmartCurrentLimit? = null,
    secondaryCurrentLimit: SecondaryCurrentLimit? = null,
    softLimits: MutableMap<CANSparkBase.SoftLimitDirection, Angle> = mutableMapOf(),
): SparkConfigurationBase(
    idleMode, inverted, voltageCompensationNominalVoltage, canTimeout, closedLoopRampRate, openLoopRampRate,
    controlFramePeriod, periodicFramePeriods, smartCurrentLimit, secondaryCurrentLimit, softLimits
)




/**
 * A wrapper around REV's [CANSparkFlex], with support for Kmeasure units
 * and integration with the rest of the library.
 */
public class ChargerSparkFlex(deviceId: Int) :
    CANSparkFlex(deviceId, MotorType.kBrushless), SmartEncoderMotorController, HardwareConfigurable<SparkFlexConfiguration> {

    override var encoder: SparkEncoderAdaptor = getEncoder(SparkFlexEncoderType.Regular())
        private set

    private fun getEncoder(encoderType: SparkFlexEncoderType) = when (encoderType){
        is SparkFlexEncoderType.Regular -> SparkEncoderAdaptor(
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

        is SparkFlexEncoderType.External -> SparkEncoderAdaptor(
            super.getExternalEncoder(
                encoderType.category,
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

        is SparkFlexEncoderType.Absolute -> SparkEncoderAdaptor(
            super.getAbsoluteEncoder(encoderType.category).apply{
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




    private val pidHandler = SparkPIDHandler(this)

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

    override fun configure(configuration: SparkFlexConfiguration) {
        configuration.encoderType?.let{ encoderType ->
            encoder = getEncoder(encoderType)
        }
        // chargerlib defined function used for safe configuration.
        safeConfigure(
            deviceName = "ChargerSparkFlex(id = $deviceId)",
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