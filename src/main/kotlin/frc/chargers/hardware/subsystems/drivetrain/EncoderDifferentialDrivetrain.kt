package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.drivetrain.DiffDriveConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.posemonitors.DifferentialPoseMonitor
import frc.chargers.hardware.subsystemutils.differentialdrive.DiffDriveIO
import frc.chargers.hardware.subsystemutils.differentialdrive.DiffDriveIOReal
import frc.chargers.hardware.subsystemutils.differentialdrive.DiffDriveIOSim
import frc.chargers.hardware.subsystemutils.differentialdrive.DiffDriveControl
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger.recordOutput

public fun simulatedDrivetrain(
    simMotors: DifferentialDrivetrainSim.KitbotMotor,
    constants: DiffDriveConstants = DiffDriveConstants.andymark(),
    controlScheme: DiffDriveControl = DiffDriveControl.None
): EncoderDifferentialDrivetrain = EncoderDifferentialDrivetrain(
    DiffDriveIOSim(logInputs = LoggableInputsProvider(namespace = "Drivetrain(Differential)"),simMotors),
    constants, controlScheme
)

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrain(
    leftMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    constants: DiffDriveConstants = DiffDriveConstants.andymark(),
    controlScheme: DiffDriveControl = DiffDriveControl.None,
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, constants, controlScheme,
        configuration = SparkMaxConfiguration().apply(configure))

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrain(
    leftMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    constants: DiffDriveConstants = DiffDriveConstants.andymark(),
    controlScheme: DiffDriveControl = DiffDriveControl.None,
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, constants, controlScheme,
        configuration = TalonFXConfiguration().apply(configure))

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : HardwareConfiguration> EncoderDifferentialDrivetrain(
    leftMotors: EncoderMotorControllerGroup<C>,
    rightMotors: EncoderMotorControllerGroup<C>,
    constants: DiffDriveConstants = DiffDriveConstants.andymark(),
    controlScheme: DiffDriveControl = DiffDriveControl.None,
    configuration: C
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        lowLevel = DiffDriveIOReal(
            logInputs = LoggableInputsProvider(namespace = "Drivetrain(Differential)"),
            leftMotors = leftMotors.apply { configure(configuration) },
            rightMotors = rightMotors.apply { configure(configuration) }
        ),
        constants, controlScheme
    )



public class EncoderDifferentialDrivetrain(
    lowLevel: DiffDriveIO,
    public val constants: DiffDriveConstants = DiffDriveConstants.andymark(),
    public val controlScheme: DiffDriveControl = DiffDriveControl.None,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider, DiffDriveIO by lowLevel {
    private val wheelRadius = constants.wheelDiameter / 2

    internal val wheelTravelPerMotorRadian = constants.gearRatio * wheelRadius

    private val leftController = UnitSuperPIDController(
        controlScheme.leftVelocityPID,
        {leftVelocity},
        target = AngularVelocity(0.0),
        selfSustain = true,
        feedforward = controlScheme.leftFF,
    )

    private val rightController = UnitSuperPIDController(
        controlScheme.rightVelocityPID,
        {rightVelocity},
        target = AngularVelocity(0.0),
        selfSustain = true,
        feedforward = controlScheme.rightPID
    )



    init{
        inverted = constants.invertMotors
    }

    /**
     * The pose estimator of the differential drivetrain.
     */
    public val poseEstimator: DifferentialPoseMonitor = DifferentialPoseMonitor(
        *poseSuppliers,
        gyro = gyro, startingPose = startingPose
    )

    /**
     * The kinematics of the drivetrain.
     *
     * @see DifferentialDriveKinematics
     */
    public val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
        constants.width.inUnit(meters)
    )

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
        get() = a[leftWheelTravel,rightWheelTravel].average() * wheelTravelPerMotorRadian

    /**
     * The current linear velocity of the robot.
     */
    public val velocity: Velocity
        get() = a[leftVelocity,rightVelocity].average() * wheelTravelPerMotorRadian

    /**
     * The current heading (the direction the robot is facing).
     *
     * This value is calculated using the encoders, not a gyroscope or accelerometer,
     * so note that it may become inaccurate if the wheels slip. If available, consider
     * using a [frc.chargers.hardware.sensors.imu.NavX] or similar device to calculate heading instead.
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
        get() = wheelTravelPerMotorRadian * (rightWheelTravel - leftWheelTravel) / constants.width


    override fun tankDrive(leftPower: Double, rightPower: Double) {
        setVoltages(leftPower * 12.volts, rightPower * 12.volts)
    }

    public fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): Unit =
        velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationSpeed))

    public fun velocityDrive(speeds: ChassisSpeeds){
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        velocityDrive(
            wheelSpeeds.leftMetersPerSecond.ofUnit(meters/seconds),
            wheelSpeeds.rightMetersPerSecond.ofUnit(meters/seconds)
        )
    }

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity){
        leftController.target = leftSpeed / (constants.gearRatio * constants.wheelDiameter)
        rightController.target = rightSpeed / (constants.gearRatio * constants.wheelDiameter)
        setVoltages(
            left = leftController.calculateOutput(),
            right = rightController.calculateOutput()
        )
    }

    override fun periodic(){
        recordOutput("Drivetrain(Differential)/distanceTraveledMeters",distanceTraveled.inUnit(meters))
        recordOutput("Drivetrain(Differential)/calculatedHeadingRad", heading.inUnit(radians))
    }

}