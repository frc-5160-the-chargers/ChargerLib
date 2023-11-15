package frc.chargers.hardware.motorcontrol.rev

import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.chargers.hardware.inputdevices.warnIfInSimulation
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration

/**
 * Creates a Spark motor controller, powered by a redline/CIM motor.
 *
 * @see [ChargerSpark]
 */
public inline fun redlineSpark(channel: Int, configure: SparkConfiguration.() -> Unit): ChargerSpark =
    ChargerSpark(channel).also{
        it.configure(SparkConfiguration().apply(configure))
    }

/**
 * Represents a Spark motor controller.
 * Includes everything in the WPILib Spark class,
 * but has additional features to mesh better with the rest
 * of this library.
 */
public class ChargerSpark(channel: Int) : Spark(channel), MotorConfigurable<SparkConfiguration> {
    init{
        warnIfInSimulation("ChargerSpark(channel = $channel)")
    }
    override fun configure(configuration: SparkConfiguration) {
        configuration.inverted?.let(::setInverted)
        println("Spark has been configured.")
    }
}

/**
 * A data class representing all possible configuration parameters
 * of a ChargerSpark.
 *
 * @see ChargerSpark
 */
public data class SparkConfiguration(var inverted: Boolean? = null) : MotorConfiguration
