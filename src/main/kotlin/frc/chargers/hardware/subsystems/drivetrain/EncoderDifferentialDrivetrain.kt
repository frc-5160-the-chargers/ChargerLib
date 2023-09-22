package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import frc.chargers.utils.Measurement
import frc.chargers.utils.WheelRatioProvider
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.processValue
import frc.chargers.wpilibextensions.motorcontrol.setVoltage

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
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, leftVelocityConstants, leftMotorFF, rightVelocityConstants, rightMotorFF, startingPose,SparkMaxConfiguration().apply(configure))

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
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, leftVelocityConstants, leftMotorFF, rightVelocityConstants, rightMotorFF, startingPose,TalonFXConfiguration().apply(configure))

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
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    startingPose: UnitPose2d = UnitPose2d(),
    configuration: C
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        leftMotors = leftMotors.apply { configure(configuration) },
        rightMotors = rightMotors.apply { configure(configuration) },
        invertMotors = invertMotors,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        width = width,
        leftVelocityConstants = leftVelocityConstants,
        leftMotorFF = leftMotorFF,
        rightVelocityConstants = rightVelocityConstants,
        rightMotorFF = rightMotorFF,
        startingPose = startingPose
    )

/**
 * An implementation of a [DifferentialDrivetrain] for use with
 * motors with encoders, allowing access to current velocity,
 * total distance driven, and heading.
 *
 * @see DifferentialDrivetrain
 */
public class EncoderDifferentialDrivetrain(
    private val leftMotors: NonConfigurableEncoderMotorControllerGroup,
    private val rightMotors: NonConfigurableEncoderMotorControllerGroup,
    invertMotors: Boolean = false,
    override val gearRatio: Double = DEFAULT_GEAR_RATIO,
    override val wheelDiameter: Length,
    private val width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    private val startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier
) : BasicDifferentialDrivetrain(leftMotors, rightMotors, invertMotors),
    HeadingProvider, RobotPoseSupplier, WheelRatioProvider {
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
        {leftMotors.encoder.angularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = true,
        feedforward = leftMotorFF
    )

    private val rightController = UnitSuperPIDController(
        rightVelocityConstants,
        {rightMotors.encoder.angularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = true,
        feedforward = rightMotorFF
    )

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity){
        leftController.target = leftSpeed / (gearRatio * wheelDiameter)
        rightController.target = rightSpeed / (gearRatio * wheelDiameter)
        leftMotors.setVoltage(leftController.calculateOutput())
        rightMotors.setVoltage(rightController.calculateOutput())
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
        heading.asRotation2d(),
        (leftMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
        (rightMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
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
            (leftMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            (rightMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            pose.inUnit(meters)
        )
    }

    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPosition(
            heading.asRotation2d(),
            (leftMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            (rightMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
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

    override fun periodic() {
        poseEstimator.update(
            heading.asRotation2d(),
            (leftMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters),
            (rightMotors.encoder.angularPosition * wheelTravelPerMotorRadian).inUnit(meters)
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
    }



}

