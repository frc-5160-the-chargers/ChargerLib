package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.constants.drivetrain.TankDriveConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.posemonitors.DifferentialPoseMonitor
import frc.chargers.hardware.subsystemutils.differentialdrive.DifferentialDriveIO
import frc.chargers.hardware.subsystemutils.differentialdrive.DifferentialDriveIOReal
import frc.chargers.hardware.subsystemutils.differentialdrive.DifferentialDriveIOSim
import frc.chargers.hardware.subsystemutils.differentialdrive.TankDriveControl
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import org.littletonrobotics.junction.Logger

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

public fun simulatedDrivetrain(
    simMotors: DifferentialDrivetrainSim.KitbotMotor,
    loopPeriod: Time = 20.milli.seconds,
    constants: TankDriveConstants = TankDriveConstants.andymark(),
    controlScheme: TankDriveControl = TankDriveControl()
): EncoderDifferentialDrivetrain = EncoderDifferentialDrivetrain(
    DifferentialDriveIOSim(simMotors,loopPeriod),
    constants, controlScheme
)

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrain(
    leftMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    constants: TankDriveConstants,
    controlScheme: TankDriveControl,
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
    constants: TankDriveConstants = TankDriveConstants.andymark(),
    controlScheme: TankDriveControl = TankDriveControl(),
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, constants, controlScheme,
        configuration = TalonFXConfiguration().apply(configure))

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : MotorConfiguration> EncoderDifferentialDrivetrain(
    leftMotors: EncoderMotorControllerGroup<C>,
    rightMotors: EncoderMotorControllerGroup<C>,
    constants: TankDriveConstants = TankDriveConstants.andymark(),
    controlScheme: TankDriveControl = TankDriveControl(),
    configuration: C
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        io = DifferentialDriveIOReal(
            leftMotors = leftMotors.apply { configure(configuration) },
            rightMotors = rightMotors.apply { configure(configuration) }
        ),
        constants, controlScheme
    )



public class EncoderDifferentialDrivetrain(
    private val io: DifferentialDriveIO,
    private val constants: TankDriveConstants = TankDriveConstants.andymark(),
    controlScheme: TankDriveControl = TankDriveControl(),
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider{

    internal val inputs = DifferentialDriveIO.Inputs()

    // lazy initializer to prevent potentital NPE's and other problems
    // associated with leaking "this"
    /**
     * The pose estimator of the differential drivetrain.
     */
    public val poseEstimator: DifferentialPoseMonitor = DifferentialPoseMonitor(
        *poseSuppliers,
        gyro = gyro, startingPose = startingPose
    )

    init{
        io.inverted = constants.invertMotors
    }





    override fun periodic(){
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Drivetrain(Differential)",inputs)
        
        leftController.calculateOutput()
        rightController.calculateOutput()

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


    private val wheelRadius = constants.wheelDiameter / 2
    internal val wheelTravelPerMotorRadian = constants.gearRatio * wheelRadius

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
                (inputs.rightAngularPosition - inputs.leftAngularPosition) / constants.width


    /**
     * The kinematics of the drivetrain.
     *
     * @see DifferentialDriveKinematics
     */
    public val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
        constants.width.inUnit(meters)
    )

    private val leftController = UnitSuperPIDController(
        controlScheme.leftVelocityConstants,
        {inputs.leftAngularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = false,
        feedforward = controlScheme.leftMotorFF
    )

    private val rightController = UnitSuperPIDController(
        controlScheme.rightVelocityConstants,
        {inputs.rightAngularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = false,
        feedforward = controlScheme.rightMotorFF
    )

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity){
        leftController.target = leftSpeed / (constants.gearRatio * constants.wheelDiameter)
        rightController.target = rightSpeed / (constants.gearRatio * constants.wheelDiameter)
        io.setVoltages(
            left = leftController.calculateOutput(),
            right = rightController.calculateOutput()
        )
    }

    public fun velocityDrive(speeds: ChassisSpeeds){
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        velocityDrive(
            wheelSpeeds.leftMetersPerSecond.ofUnit(meters/seconds),
            wheelSpeeds.rightMetersPerSecond.ofUnit(meters/seconds)
        )
    }






    

}