package frc.robot.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.robot.hardware.interfaces.MotorConfigurable
import frc.robot.hardware.interfaces.MotorConfiguration

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
