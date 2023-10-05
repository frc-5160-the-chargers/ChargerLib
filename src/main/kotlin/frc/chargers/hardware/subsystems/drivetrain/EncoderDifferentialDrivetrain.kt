package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.utils.Measurement
import frc.chargers.utils.WheelRatioProvider
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.processValue
import org.littletonrobotics.junction.Logger

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

public fun simulatedDrivetrain(
    simMotors: DifferentialDrivetrainSim.KitbotMotor,
    loopPeriod: Time = 20.milli.seconds,
    invertMotors: Boolean = false,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
): EncoderDifferentialDrivetrain = EncoderDifferentialDrivetrain(
    EncoderDifferentialDrivetrainIOSim(simMotors,loopPeriod),
    invertMotors, gearRatio, wheelDiameter, width, null, leftVelocityConstants, leftMotorFF, rightVelocityConstants, rightMotorFF, startingPose, *poseSuppliers
)

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
    gyro: HeadingProvider? = null,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, gyro, leftVelocityConstants, leftMotorFF , rightVelocityConstants, rightMotorFF, startingPose, *poseSuppliers,
        configuration = SparkMaxConfiguration().apply(configure))

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
    gyro: HeadingProvider? = null,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, gyro, leftVelocityConstants, leftMotorFF , rightVelocityConstants, rightMotorFF, startingPose, *poseSuppliers,
        configuration = TalonFXConfiguration().apply(configure))

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
    gyro: HeadingProvider? = null,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
    configuration: C
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        io = EncoderDifferentialDrivetrainIOReal(
            leftMotors = leftMotors.apply { configure(configuration) },
            rightMotors = rightMotors.apply { configure(configuration) }
        ),
        invertMotors = invertMotors,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        width = width,
        gyro = gyro,
        leftVelocityConstants, leftMotorFF, rightVelocityConstants, rightMotorFF, startingPose, *poseSuppliers
    )



public class EncoderDifferentialDrivetrain(
    private val io: EncoderDifferentialDrivetrainIO,
    invertMotors: Boolean = false,
    override val gearRatio: Double = DEFAULT_GEAR_RATIO,
    override val wheelDiameter: Length,
    private val width: Distance,
    private val gyro: HeadingProvider? = null,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    private val startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider, WheelRatioProvider, RobotPoseSupplier {
    init{
        io.inverted = invertMotors
    }
    private val inputs = EncoderDifferentialDrivetrainIO.Inputs()

    private fun mostAccurateHeadingValue(): Angle = gyro?.heading ?: heading

    /**
     * A representation of the field, for simulation and pose-visualizing purposes.
     */
    public val field: Field2d = Field2d().also{
        SmartDashboard.putData("Field",it)
    }

    override fun periodic(){
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Drivetrain(Differential)",inputs)
        Logger.getInstance().recordOutput("Drivetrain(Differential)/Pose2d(meters)", robotPose.inUnit(meters))
        
        leftController.calculateOutput()
        rightController.calculateOutput()

        poseEstimator.update(
            mostAccurateHeadingValue().asRotation2d(),
            (inputs.leftAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            (inputs.rightAngularPosition * wheelTravelPerMotorRadian).inUnit(meters)
        )

        allPoseSuppliers.forEach{
            val poseMeasurement = it.robotPoseMeasurement

            it.poseStandardDeviation.processValue(
                whenValueExists = { stdDev ->
                    poseEstimator.addVisionMeasurement(
                        poseMeasurement.value.inUnit(meters),
                        poseMeasurement.timestamp.inUnit(seconds),
                        stdDev.getVector()
                    )
                },
                whenDefault = {
                    poseEstimator.addVisionMeasurement(
                        poseMeasurement.value.inUnit(meters),
                        poseMeasurement.timestamp.inUnit(seconds)
                    )
                }
            )
        }

        // robotPose is a property of the RobotPoseSupplier interface; value retreived from the
        // robotPoseMeasurement getter(seen in this class).
        field.robotPose = robotPose.inUnit(meters)
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


    /**
     * The kinematics of the drivetrain.
     *
     * @see DifferentialDriveKinematics
     */
    public val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
        width.inUnit(meters)
    )

    private val leftController = UnitSuperPIDController(
        leftVelocityConstants,
        {inputs.leftAngularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = false,
        feedforward = leftMotorFF
    )

    private val rightController = UnitSuperPIDController(
        rightVelocityConstants,
        {inputs.rightAngularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = false,
        feedforward = rightMotorFF
    )

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity){
        leftController.target = leftSpeed / (gearRatio * wheelDiameter)
        rightController.target = rightSpeed / (gearRatio * wheelDiameter)
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

    private val poseEstimator: DifferentialDrivePoseEstimator = DifferentialDrivePoseEstimator(
        kinematics,
        mostAccurateHeadingValue().asRotation2d(),
        (inputs.leftAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
        (inputs.rightAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
        startingPose.inUnit(meters),
    )

    private val allPoseSuppliers: MutableList<RobotPoseSupplier> = poseSuppliers.toMutableList()


    /**
     * adds a variable amount of pose suppliers to the drivetrain.
     * these pose suppliers will fuse their poses into the pose estimator for more accurate measurements.
     */
    public fun addPoseSuppliers(vararg poseSuppliers: RobotPoseSupplier){
        allPoseSuppliers.addAll(poseSuppliers)
    }

    public fun resetPose(pose: UnitPose2d, gyroAngle: Angle){
        poseEstimator.resetPosition(
            gyroAngle.asRotation2d(),
            (inputs.leftAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            (inputs.rightAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            pose.inUnit(meters)
        )
    }

    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPosition(
            mostAccurateHeadingValue().asRotation2d(),
            (inputs.leftAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            (inputs.rightAngularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            pose.inUnit(meters)
        )
    }

    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() = Measurement(
            poseEstimator.estimatedPosition.ofUnit(meters),
            fpgaTimestamp(),
            true
        )

    override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default
    

}