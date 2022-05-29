package frc.robot.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.*
import frc.robot.hardware.interfaces.MotorConfiguration
import frc.robot.hardware.motorcontrol.*

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrain(
    leftMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    invertMotors: Boolean = false, gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length, configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, SparkMaxConfiguration().apply(configure))

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrain(
    leftMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    invertMotors: Boolean = false, gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    configure: TalonFXConfiguration.() -> Unit): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, TalonFXConfiguration().apply(configure))

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : MotorConfiguration> EncoderDifferentialDrivetrain(leftMotors: EncoderMotorControllerGroup<C>, rightMotors: EncoderMotorControllerGroup<C>, invertMotors: Boolean = false, gearRatio: Double, wheelDiameter: Length, configuration: C): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        leftMotors = leftMotors.apply { configure(configuration) },
        rightMotors = rightMotors.apply { configure(configuration) },
        invertMotors = invertMotors,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter
    )

/**
 * An implementation of a [DifferentialDrivetrain] for use with
 * motors with encoders, allowing access to current velocity and
 * total distance driven.
 *
 * @see DifferentialDrivetrain
 */
public open class EncoderDifferentialDrivetrain(
    leftMotors: NonConfigurableEncoderMotorControllerGroup,
    rightMotors: NonConfigurableEncoderMotorControllerGroup,
    invertMotors: Boolean = false,
    protected val gearRatio: Double = DEFAULT_GEAR_RATIO,
    protected val wheelDiameter: Length
) : BasicDifferentialDrivetrain(leftMotors, rightMotors, invertMotors) {
    private val overallEncoder = AverageEncoder(leftMotors, rightMotors)

    public val distanceTraveled: Distance
        get() =
            overallEncoder.angularPosition *
                    gearRatio *
                    wheelDiameter / 2

    public val velocity: Velocity
        get() =
            overallEncoder.angularVelocity *
                    gearRatio *
                    wheelDiameter / 2
}