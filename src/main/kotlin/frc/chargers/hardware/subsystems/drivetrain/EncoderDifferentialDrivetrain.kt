package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.*
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.encoders.AverageEncoder

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
    wheelDiameter: Length,
    width: Distance,
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, SparkMaxConfiguration().apply(configure))

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrain(
    leftMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    invertMotors: Boolean = false,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, TalonFXConfiguration().apply(configure))

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : MotorConfiguration> EncoderDifferentialDrivetrain(
    leftMotors: EncoderMotorControllerGroup<C>,
    rightMotors: EncoderMotorControllerGroup<C>,
    invertMotors: Boolean = false, gearRatio: Double,
    wheelDiameter: Length,
    width: Distance,
    configuration: C
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        leftMotors = leftMotors.apply { configure(configuration) },
        rightMotors = rightMotors.apply { configure(configuration) },
        invertMotors = invertMotors,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        width = width
    )

/**
 * An implementation of a [DifferentialDrivetrain] for use with
 * motors with encoders, allowing access to current velocity,
 * total distance driven, and heading.
 *
 * @see DifferentialDrivetrain
 */
public open class EncoderDifferentialDrivetrain(
    private val leftMotors: NonConfigurableEncoderMotorControllerGroup,
    private val rightMotors: NonConfigurableEncoderMotorControllerGroup,
    invertMotors: Boolean = false,
    protected val gearRatio: Double = DEFAULT_GEAR_RATIO,
    protected val wheelDiameter: Length,
    protected val width: Distance
) : BasicDifferentialDrivetrain(leftMotors, rightMotors, invertMotors), HeadingProvider {
    private val overallEncoder = AverageEncoder(leftMotors, rightMotors)

    private val wheelRadius = wheelDiameter / 2
    private val wheelTravelPerMotorRadian = gearRatio * wheelRadius

    /**
     * The total linear distance traveled from the zero point of the encoders.
     *
     * This value by itself is not particularly meaningful as it may be fairly large,
     * positive or negative, based on previous rotations of the motors, including
     * from previous times the robot has been enabled.
     *
     * Thus, it's more common to use this property to determine *change* in position.
     * If the initial value of this property is stored, the distance traveled since
     * that initial point can easily be determined by subtracting the initial position
     * from the current position.
     */
    public val distanceTraveled: Distance
        get() =
            overallEncoder.angularPosition *
                    wheelTravelPerMotorRadian

    /**
     * The current linear velocity of the robot.
     */
    public val velocity: Velocity
        get() =
            overallEncoder.angularVelocity *
                    wheelTravelPerMotorRadian

    /**
     * The current heading (the direction the robot is facing).
     *
     * This value is calculated using the encoders, not a gyroscope or accelerometer,
     * so note that it may become inaccurate if the wheels slip. If available, consider
     * using a [frc.chargers.hardware.sensors.NavX] or similar device to calculate heading instead.
     *
     * This value by itself is not particularly meaningful as it may be fairly large,
     * positive or negative, based on previous rotations of the motors, including
     * from previous times the robot has been enabled.
     *
     * Thus, it's more common to use this property to determine *change* in heading.
     * If the initial value of this property is stored, the amount of rotation since
     * that initial point can easily be determined by subtracting the initial heading
     * from the current heading.
     *
     * @see HeadingProvider
     */
    public override val heading: Angle
        get() = wheelTravelPerMotorRadian *
                (rightMotors.encoder.angularPosition - leftMotors.encoder.angularPosition) / width
}