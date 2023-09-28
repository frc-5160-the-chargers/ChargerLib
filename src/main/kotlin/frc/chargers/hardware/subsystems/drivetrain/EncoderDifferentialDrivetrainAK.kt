package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.utils.WheelRatioProvider
import frc.chargers.utils.a
import org.littletonrobotics.junction.Logger


public fun simulatedDrivetrain(
    invertMotors: Boolean = false,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance
): EncoderDifferentialDrivetrainAK = EncoderDifferentialDrivetrainAK(
    EncoderDifferentialDrivetrainIOSim(),
    invertMotors, gearRatio, wheelDiameter, width
)

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrainAK(
    leftMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    invertMotors: Boolean = false, gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrainAK =
    EncoderDifferentialDrivetrainAK(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width,
        SparkMaxConfiguration().apply(configure))

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrainAK(
    leftMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    invertMotors: Boolean = false,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrainAK =
    EncoderDifferentialDrivetrainAK(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width,
        TalonFXConfiguration().apply(configure))

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : MotorConfiguration> EncoderDifferentialDrivetrainAK(
    leftMotors: EncoderMotorControllerGroup<C>,
    rightMotors: EncoderMotorControllerGroup<C>,
    invertMotors: Boolean = false, gearRatio: Double,
    wheelDiameter: Length,
    width: Distance,
    configuration: C
): EncoderDifferentialDrivetrainAK =
    EncoderDifferentialDrivetrainAK(
        io = EncoderDifferentialDrivetrainIOReal(
            leftMotors = leftMotors.apply { configure(configuration) },
            rightMotors = rightMotors.apply { configure(configuration) }
        ),
        invertMotors = invertMotors,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        width = width
    )



public class EncoderDifferentialDrivetrainAK(
    private val io: EncoderDifferentialDrivetrainIO,
    invertMotors: Boolean = false,
    override val gearRatio: Double = DEFAULT_GEAR_RATIO,
    override val wheelDiameter: Length,
    private val width: Distance,
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider, WheelRatioProvider {
    init{
        io.inverted = invertMotors
    }
    private val inputs = EncoderDifferentialDrivetrainIO.Inputs()

    override fun periodic(){
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("DrivetrainDifferential",inputs)
    }

    override fun tankDrive(leftPower: Double, rightPower: Double) {
        io.setVoltages(leftPower * 12.volts, rightPower * 12.volts)
    }

    override fun arcadeDrive(power: Double, rotation: Double) {
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(power,rotation,false)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    override fun curvatureDrive(power: Double, steering: Double) {
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(power,steering,true)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    override fun stop() {
        io.setVoltages(0.volts,0.volts)
    }


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
            a[inputs.leftAngularPosition,inputs.rightAngularPosition].average() * wheelTravelPerMotorRadian

    /**
     * The current linear velocity of the robot.
     */
    public val velocity: Velocity
        get() =
            a[inputs.leftAngularVelocity,inputs.rightAngularVelocity].average() * wheelTravelPerMotorRadian

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
                (inputs.rightAngularPosition - inputs.leftAngularPosition) / width

}