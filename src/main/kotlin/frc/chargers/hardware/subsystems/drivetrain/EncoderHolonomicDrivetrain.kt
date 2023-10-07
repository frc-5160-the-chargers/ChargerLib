package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.chargers.advantagekitextensions.loggedwrappers.LoggedHeadingProvider
import frc.chargers.hardware.swerve.SwerveDriveMotors
import frc.chargers.hardware.swerve.SwerveEncoders
import frc.chargers.hardware.swerve.SwerveTurnMotors
import frc.chargers.hardware.swerve.control.SwerveAngleControl
import frc.chargers.hardware.swerve.control.SwerveSpeedControl
import frc.chargers.hardware.swerve.module.*
import frc.chargers.hardware.swerve.module.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.hardware.swerve.module.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.utils.a
import frc.chargers.utils.math.units.Inertia
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.kinematics.*
import frc.chargers.wpilibextensions.kinematics.swerve.*
import frc.chargers.wpilibextensions.processValue
import org.littletonrobotics.junction.Logger

@PublishedApi
internal val DEFAULT_MAX_MODULE_SPEED: Velocity = 4.5.ofUnit(meters/seconds)

/**
 * A convenience function used to create an [EncoderHolonomicDrivetrain], with [ModuleIOSim] as the Module IO.
 */
public fun simEncoderHolonomicDrivetrain(
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    loopPeriod: Time = 20.milli.seconds,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    turnControl: SwerveAngleControl,
    velocityControl: SwerveSpeedControl,
    gyro: HeadingProvider? = null,
    maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    startingPose: UnitPose2d = UnitPose2d(),
    fieldRelativeDrive: Boolean = true,
    staticVoltageStall: Boolean = false,
    vararg poseSuppliers: RobotPoseSupplier
): EncoderHolonomicDrivetrain = EncoderHolonomicDrivetrain(
    topLeft = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), turnControl, velocityControl, staticVoltageStall
    ),
    topRight = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), turnControl, velocityControl, staticVoltageStall
    ),
    bottomLeft = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), turnControl, velocityControl, staticVoltageStall
    ),
    bottomRight = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), turnControl, velocityControl, staticVoltageStall
    ),
    gyro, maxModuleSpeed, wheelDiameter, trackWidth, wheelBase, startingPose, fieldRelativeDrive, *poseSuppliers
)


/**
 * A Convenience function for creating an [EncoderHolonomicDrivetrain] with [ModuleIOReal] as the swerve module IO.
 *
 * It uses the [SwerveTurnMotors], [SwerveEncoders] and [SwerveDriveMotors] classes
 * instead of defining individual swerve modules.
 *
 * This allows the motors and encoders to all be group-configured.
 */
public fun realEncoderHolonomicDrivetrain(
    turnMotors: SwerveTurnMotors,
    turnEncoders: SwerveEncoders,
    driveMotors: SwerveDriveMotors,
    turnControl: SwerveAngleControl,
    velocityControl: SwerveSpeedControl,
    gyro: HeadingProvider,
    maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    startingPose: UnitPose2d = UnitPose2d(),
    fieldRelativeDrive: Boolean = true,
    staticVoltageStall: Boolean = false,
    vararg poseSuppliers: RobotPoseSupplier
): EncoderHolonomicDrivetrain{
    val topLeft = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.topLeft,
            turnEncoder = turnEncoders.topLeft,
            driveMotor = driveMotors.topLeft,
            driveGearRatio, turnGearRatio
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    val topRight = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.topRight,
            turnEncoder = turnEncoders.topRight,
            driveMotor = driveMotors.topRight,
            driveGearRatio, turnGearRatio
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    val bottomLeft = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.bottomLeft,
            turnEncoder = turnEncoders.bottomLeft,
            driveMotor = driveMotors.bottomLeft,
            driveGearRatio,turnGearRatio
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    val bottomRight = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.bottomRight,
            turnEncoder = turnEncoders.bottomRight,
            driveMotor = driveMotors.bottomRight,
            driveGearRatio, turnGearRatio
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    return EncoderHolonomicDrivetrain(
        topLeft, topRight, bottomLeft, bottomRight,
        gyro, maxModuleSpeed, wheelDiameter,
        trackWidth, wheelBase, startingPose, fieldRelativeDrive, *poseSuppliers
    )
}




// note: trackwidth is horizontal and wheelBase is vertical
// second note: DEFAULT_GEAR_RATIO is defined in encoderdifferentialdrivetrain.
/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 */
public class EncoderHolonomicDrivetrain(
    public val topLeft: SwerveModule,
    public val topRight: SwerveModule,
    public val bottomLeft: SwerveModule,
    public val bottomRight: SwerveModule,
    private val gyro: HeadingProvider? = null,
    public val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    public val wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    public val startingPose: UnitPose2d = UnitPose2d(),
    // do drivetrain.fieldRelativeDrive = false to turn this option off.
    public var fieldRelativeDrive: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier
): SubsystemBase(), RobotPoseSupplier, HeadingProvider{
    /**
     * A representation of the field, for simulation and pose-visualizing purposes.
     */
    public val field: Field2d = Field2d().also{
        SmartDashboard.putData("Field",it)
    }

    /**
     * The kinematics class for the drivetrain.
     * Uses [SuperSwerveDriveKinematics], a custom wrapper class around
     * the [edu.wpi.first.math.kinematics.SwerveDriveKinematics] class.
     */
    public val kinematics: SuperSwerveDriveKinematics = SuperSwerveDriveKinematics(
        UnitTranslation2d(trackWidth/2,wheelBase/2),
        UnitTranslation2d(trackWidth/2,-wheelBase/2),
        UnitTranslation2d(-trackWidth/2,wheelBase/2),
        UnitTranslation2d(-trackWidth/2,-wheelBase/2)
    )







    /*
    HeadingProvider-Based functions and getters below.
     */
    private val headingProvider: LoggedHeadingProvider = if (gyro == null){
        LoggedHeadingProvider.fromSwerveKinematics(kinematics){currentModulePositions}
    }else{
        LoggedHeadingProvider(gyro)
    }


    /**
     * The heading of the drivetrain.
     *
     * If a gyro is specified, it is the same as the gyro.
     *
     * Otherwise, it uses the drivetrain to calculate heading
     */
    override val heading: Angle
        get() = headingProvider.heading






    /*
    Control Mode-based functions:
    OPEN_LOOP indicates percent-out(power) based drive,
    while CLOSED_LOOP uses PID and feedtopLeft.setPower(0.5)forward for velocity-based driving.
     */
    private enum class ControlMode{
        OPEN_LOOP,CLOSED_LOOP
    }
    private var currentControlMode = ControlMode.CLOSED_LOOP







    /*
    Odometry and pose estimation-based functions, getters and properties are here.
    Utilizes the PoseSupplier interface;
    which requires implementation of pose measurement and pose reliability getters.
    pose reliability uses the StandardDeviation class.
     */
    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        headingProvider.heading.asRotation2d(),
        a[
            topLeft.getModulePosition(wheelDiameter),
            topRight.getModulePosition(wheelDiameter),
            bottomLeft.getModulePosition(wheelDiameter),
            bottomRight.getModulePosition(wheelDiameter)
        ],
        startingPose.inUnit(meters)
    )

    private val allPoseSuppliers: MutableList<RobotPoseSupplier> = poseSuppliers.toMutableList()

    /**
     * adds a variable amount of pose suppliers to the drivetrain.
     * these pose suppliers will fuse their poses into the pose estimator for more accurate measurements.
     */
    public fun addPoseSuppliers(vararg poseSuppliers: RobotPoseSupplier){
        allPoseSuppliers.addAll(poseSuppliers)
    }

    /**
     * The robot pose measurement that the subsystem provides.
     */
    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() = Measurement(
            poseEstimator.estimatedPosition.ofUnit(meters),
            fpgaTimestamp(),
            true
        )

    override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default

    /**
     * Resets the odometry-provided pose of the drivetrain.
     */
    public fun resetPose(pose: UnitPose2d, gyroAngle: Angle = headingProvider.heading){
        poseEstimator.resetPosition(
            gyroAngle.asRotation2d(),
            currentModulePositions.toArray(),
            pose.inUnit(meters)
        )
    }




    private val wheelRadius = wheelDiameter / 2.0


    /*
    Encoder-based functions below. They use similar tactics to the EncoderDifferentialDrivetrain class.
     */
    private val moduleArray = a[topLeft,topRight,bottomLeft,bottomRight]
    
    // gear ratio compensated
    private fun averageEncoderPosition() =
        moduleArray.map{it.wheelPosition}.average()
    private fun averageEncoderVelocity() =
        moduleArray.map{it.currentVelocity}.average()
    
    private val distanceOffset: Distance = averageEncoderPosition() * wheelRadius

    public val distanceTraveled: Distance
        get() =
            (averageEncoderPosition() *
                    wheelRadius) - distanceOffset
    public val velocity: Velocity
        get() =
            averageEncoderVelocity() *
                    wheelRadius










    /**
     * A getter/setter variable that can either fetch the current speeds
     * of each swerve module,
     * or can set the desired speeds of each swerve module.
     * It is reccomended that the [swerveDrive] and [velocityDrive] functions are used instead.
     */
    public var currentModuleStates: ModuleStateGroup
        get() = ModuleStateGroup(
            topLeftState = topLeft.getModuleState(wheelRadius),
            topRightState = topRight.getModuleState(wheelRadius),
            bottomLeftState = bottomLeft.getModuleState(wheelRadius),
            bottomRightState = bottomRight.getModuleState(wheelRadius)
        )
        set(ms){
            ms.desaturate(maxModuleSpeed)
            if (currentControlMode == ControlMode.CLOSED_LOOP){
                topLeft.setDirectionalVelocity(ms.topLeftSpeed / wheelRadius,ms.topLeftAngle,)
                topRight.setDirectionalVelocity(ms.topRightSpeed / wheelRadius,ms.topRightAngle)
                bottomLeft.setDirectionalVelocity(ms.bottomLeftSpeed / wheelRadius,ms.bottomLeftAngle)
                bottomRight.setDirectionalVelocity(ms.bottomRightSpeed / wheelRadius,ms.bottomRightAngle)
            }else{
                topLeft.setDirectionalPower((ms.topLeftSpeed/maxModuleSpeed).siValue, ms.topLeftAngle)
                topRight.setDirectionalPower((ms.topRightSpeed/maxModuleSpeed).siValue, ms.topRightAngle)
                bottomLeft.setDirectionalPower((ms.bottomLeftSpeed/maxModuleSpeed).siValue, ms.bottomLeftAngle)
                bottomRight.setDirectionalPower((ms.bottomRightSpeed/maxModuleSpeed).siValue, ms.bottomRightAngle)
            }

            Logger.getInstance().apply{
                recordOutput("topLeftDesiredSpeedMPS", ms.topLeftSpeed.inUnit(meters/seconds))
                recordOutput("topRightDesiredSpeedMPS", ms.topRightSpeed.inUnit(meters/seconds))
                recordOutput("bottomLeftDesiredSpeedMPS", ms.bottomLeftSpeed.inUnit(meters/seconds))
                recordOutput("bottomRightDesiredSpeedMPS", ms.bottomRightSpeed.inUnit(meters/seconds))

                recordOutput("topLeftDesiredAngleDeg",ms.topLeftAngle.inUnit(degrees))
                recordOutput("topRightDesiredAngleDeg",ms.topRightAngle.inUnit(degrees))
                recordOutput("bottomLeftDesiredAngleDeg",ms.bottomLeftAngle.inUnit(degrees))
                recordOutput("bottomRightDesiredAngleDeg",ms.bottomRightAngle.inUnit(degrees))
            }
        }


    /**
     * Gets the current module positions of each swerve module.
     * Returns a [ModulePositionGroup] object,
     * a wrapper around 4 [SwerveModulePosition] objects with kmeasure support.
     */
    public val currentModulePositions: ModulePositionGroup
        get() = ModulePositionGroup(
            topLeftPosition = topLeft.getModulePosition(wheelRadius),
            topRightPosition = topRight.getModulePosition(wheelRadius),
            bottomLeftPosition = bottomLeft.getModulePosition(wheelRadius),
            bottomRightPosition = bottomRight.getModulePosition(wheelRadius)
        )









    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    public val maxLinearVelocity: Velocity = kinematics.toChassisSpeeds(
        ModuleStateGroup(
            topLeftSpeed = maxModuleSpeed,
            topRightSpeed = maxModuleSpeed,
            bottomLeftSpeed = maxModuleSpeed,
            bottomRightSpeed = maxModuleSpeed,
            topLeftAngle = Angle(0.0),
            topRightAngle = Angle(0.0),
            bottomLeftAngle = Angle(0.0),
            bottomRightAngle = Angle(0.0)
        )
    ).xVelocity

    /**
     * The max angular velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential(with each being oriented at a 45 or -45 degrees angle),
     * then calculating the output using the kinematics object.
     */
    public val maxRotationalVelocity: AngularVelocity = kinematics.toChassisSpeeds(

        ModuleStateGroup(
            topLeftSpeed = maxModuleSpeed,
            topRightSpeed = -maxModuleSpeed,
            bottomLeftSpeed = maxModuleSpeed,
            bottomRightSpeed = -maxModuleSpeed,
            topLeftAngle = -45.degrees,
            topRightAngle = 45.degrees,
            bottomLeftAngle = 45.degrees,
            bottomRightAngle = -45.degrees
        )
    ).rotationSpeed







    /*
    Drive functions below:
     */

    /**
     * The default drive function for swerve.
     * accepts an xPower, yPower and rotationPower.
     *
     * Similar to open-loop control modes of other swerve drivetrains.
     */
    public fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double): Unit =
        swerveDrive(ChassisPowers(xPower,yPower,rotationPower))

    /**
     * Drives with a ChassisPowers object storing xPower, yPower, and rotationPower.
     *
     * @see swerveDrive
     */
    public fun swerveDrive(powers: ChassisPowers){
        var speeds = powers.toChassisSpeeds(maxLinearVelocity,maxRotationalVelocity)
        if(fieldRelativeDrive){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,headingProvider.heading.asRotation2d()
            )
        }
        currentControlMode = ControlMode.OPEN_LOOP
        currentModuleStates = kinematics.toSecondOrderModuleStateGroup(
            speeds.correctForDynamics(),
            headingProvider.heading
        )
        currentControlMode = ControlMode.CLOSED_LOOP

    }





    /**
     * A version of velocityDrive that uses [ChassisSpeeds] instead.
     *
     * @see velocityDrive
     */
    public fun velocityDrive(speeds: ChassisSpeeds): Unit = velocityDrive(
        speeds.xVelocity,
        speeds.yVelocity,
        speeds.rotationSpeed
    )

    /**
     * Drives the robot using certain velocities instead of certain amounts of power(or duty cycle output).
     * Relies on PID in order to be used effectively.
     *
     * This is also called closed-loop control in other swerve drivetrain subsystems.
     */
    public fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationVelocity: AngularVelocity){

        currentControlMode = ControlMode.CLOSED_LOOP
        val speeds = if(fieldRelativeDrive){
            FieldRelativeChassisSpeeds(
                xVelocity,
                yVelocity,
                rotationVelocity,
                headingProvider.heading
            )
        }else{
            ChassisSpeeds(
                xVelocity,
                yVelocity,
                rotationVelocity
            )
        }
        currentModuleStates = kinematics.toSecondOrderModuleStateGroup(
            speeds.correctForDynamics(),
            headingProvider.heading
        )
    }








    /**
     * Stops the drivetrain.
     */
    public fun stop(){
        topLeft.halt()
        topRight.halt()
        bottomLeft.halt()
        bottomRight.halt()
    }

    /**
     * Stops the drivetrain in an X.
     */
    public fun stopInX(){
        topLeft.setDirectionalPower(0.0,45.degrees)
        topRight.setDirectionalPower(0.0,-45.degrees)
        bottomLeft.setDirectionalPower(0.0,-45.degrees)
        bottomRight.setDirectionalPower(0.0,45.degrees)
    }




    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        topLeft.updateAndProcessInputs("Drivetrain(Swerve)/TopLeftSwerveModule")
        topRight.updateAndProcessInputs("Drivetrain(Swerve)/TopRightSwerveModule")
        bottomLeft.updateAndProcessInputs("Drivetrain(Swerve)/BottomLeftSwerveModule")
        bottomRight.updateAndProcessInputs("Drivetrain(Swerve)/BottomRightSwerveModule")
        headingProvider.updateAndProcessInputs("Drivetrain(Swerve)/GyroData")
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/Pose2d(meters)", robotPose.inUnit(meters))




        /*
         * Updates the pose estimator with the current module positions,
         * as well as the gyro's heading.
         */
        poseEstimator.update(
            headingProvider.heading.asRotation2d(),
            currentModulePositions.toArray()
        )

        /*
         * Updates the pose estimator with the vision measurements from the pose suppliers.
         */
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


}