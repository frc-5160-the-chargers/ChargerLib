package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.drivetrain.DiffDriveHardwareData
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.differentialdrive.lowlevel.DiffDriveIO
import frc.chargers.hardware.subsystems.differentialdrive.lowlevel.DiffDriveIOReal
import frc.chargers.hardware.subsystems.differentialdrive.lowlevel.DiffDriveIOSim
import frc.chargers.constants.drivetrain.DiffDriveControlData
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger.recordOutput

private val standardLogInputs = LoggableInputsProvider(namespace = "Drivetrain(Differential)")


/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrain(
    leftMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    controlData: DiffDriveControlData = DiffDriveControlData.None,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        leftMotors, rightMotors, hardwareData, controlData,
        DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
        gyro, startingPose, *poseSuppliers,
        configuration = SparkMaxConfiguration().apply(configure)
    )


/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrain(
    leftMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    controlData: DiffDriveControlData = DiffDriveControlData.None,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        leftMotors, rightMotors, hardwareData, controlData,
        DifferentialDrivetrainSim.KitbotMotor.kDoubleFalcon500PerSide,
        gyro, startingPose, *poseSuppliers,
        configuration = TalonFXConfiguration().apply(configure)
    )

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : HardwareConfiguration> EncoderDifferentialDrivetrain(
    leftMotors: EncoderMotorControllerGroup<C>,
    rightMotors: EncoderMotorControllerGroup<C>,
    hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    controlData: DiffDriveControlData = DiffDriveControlData.None,
    simMotors: DifferentialDrivetrainSim.KitbotMotor = DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
    configuration: C
): EncoderDifferentialDrivetrain{
    val lowLevel = if (RobotBase.isReal()){
        DiffDriveIOReal(
            standardLogInputs,
            leftMotors.apply { configure(configuration) },
            rightMotors.apply { configure(configuration) }
        )
    }else{
        DiffDriveIOSim(standardLogInputs, simMotors)
    }

    return EncoderDifferentialDrivetrain(lowLevel, hardwareData, controlData, gyro, startingPose, *poseSuppliers)
}



public class EncoderDifferentialDrivetrain(
    lowLevel: DiffDriveIO,
    public val hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    public val controlData: DiffDriveControlData = DiffDriveControlData.None,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider, DiffDriveIO by lowLevel {
    private val wheelRadius = hardwareData.wheelDiameter / 2

    internal val wheelTravelPerMotorRadian = hardwareData.gearRatio * wheelRadius

    private val leftController = SuperPIDController(
        controlData.leftVelocityPID,
        getInput = {leftVelocity},
        target = AngularVelocity(0.0),
        setpointSupplier = SetpointSupplier.Default(
            feedforward = Feedforward(controlData.leftFF)
        ),
        selfSustain = true,
    )

    private val rightController = SuperPIDController(
        controlData.rightVelocityPID,
        getInput = {rightVelocity},
        target = AngularVelocity(0.0),
        setpointSupplier = SetpointSupplier.Default(
            feedforward = Feedforward(controlData.rightFF)
        ),
        selfSustain = true,
    )



    init{
        inverted = hardwareData.invertMotors


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
        hardwareData.width.inUnit(meters)
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
     * using a [frc.chargers.hardware.sensors.imu.ChargerNavX] or similar device to calculate heading instead.
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
        get() = wheelTravelPerMotorRadian * (rightWheelTravel - leftWheelTravel) / hardwareData.width

    /**
     * Gets the current [ChassisSpeeds] of the robot.
     */
    public val currentSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(
                (leftVelocity * wheelTravelPerMotorRadian).inUnit(meters/seconds),
                (rightVelocity * wheelTravelPerMotorRadian).inUnit(meters/seconds),
            )
        )

    // arcade drive and curvature drive implementations
    // are provided in the DifferentialDrivetrain interface
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
        leftController.target = leftSpeed / (hardwareData.gearRatio * hardwareData.wheelDiameter)
        rightController.target = rightSpeed / (hardwareData.gearRatio * hardwareData.wheelDiameter)
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