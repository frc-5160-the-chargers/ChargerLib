package frc.chargers.hardware.motorcontrol.rev

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration

public inline fun redlineSpark(channel: Int, configure: SparkConfiguration.() -> Unit): ChargerSpark =
    ChargerSpark(channel).also{it.configure(SparkConfiguration().apply(configure))}

/**
 * Represents a Spark motor controller.
 * Includes everything in the WPILib Spark class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * @see edu.wpi.first.wpilibj.motorcontrol.Spark
 * @see SparkConfiguration
 */
public class ChargerSpark(channel: Int) : Spark(channel), MotorConfigurable<SparkConfiguration> {
    init{
        if (RobotBase.isSimulation()){
            error("Looks like you instantiated a motor IN SIM. Make sure to use a lazy initializer! ")
        }
    }
    override fun configure(configuration: SparkConfiguration) {
        configuration.inverted?.let(::setInverted)
    }
}

/**
 * A data class representing all possible configuration parameters
 * of a ChargerSpark.
 *
 * @see ChargerSpark
 */
public data class SparkConfiguration(var inverted: Boolean? = null) : MotorConfiguration
