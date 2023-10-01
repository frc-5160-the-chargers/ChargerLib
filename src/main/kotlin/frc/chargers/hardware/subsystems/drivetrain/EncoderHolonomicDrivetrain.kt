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
import frc.chargers.hardware.swerve.SwerveDriveMotors
import frc.chargers.hardware.swerve.SwerveEncoders
import frc.chargers.hardware.swerve.SwerveTurnMotors
import frc.chargers.hardware.swerve.control.TurnPID
import frc.chargers.hardware.swerve.control.VelocityPID
import frc.chargers.hardware.swerve.module.*
import frc.chargers.hardware.swerve.module.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.hardware.swerve.module.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.utils.WheelRatioProvider
import frc.chargers.utils.a
import frc.chargers.utils.math.units.Inertia
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.kinematics.*
import frc.chargers.wpilibextensions.kinematics.swerve.*
import frc.chargers.wpilibextensions.processValue

@PublishedApi
internal val DEFAULT_MAX_MODULE_SPEED: Velocity = 4.5.ofUnit(meters/seconds)


public fun simEncoderHolonomicDrivetrain(
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    loopPeriod: Time = 20.milli.seconds,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    turnControl: TurnPID,
    velocityControl: VelocityPID,
    gyro: HeadingProvider,
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
    gyro, maxModuleSpeed, driveGearRatio, wheelDiameter, trackWidth, wheelBase, startingPose, fieldRelativeDrive, *poseSuppliers
)


/**
 * A Convenience function for creating an [EncoderHolonomicDrivetrain]
 * Using the [SwerveTurnMotors], [SwerveEncoders] and [SwerveDriveMotors] classes
 * Instead of defining individual swerve modules.
 * This allows the motors and encoders to all be configured.
 */
public fun realEncoderHolonomicDrivetrain(
    turnMotors: SwerveTurnMotors,
    turnEncoders: SwerveEncoders,
    driveMotors: SwerveDriveMotors,
    turnControl: TurnPID,
    velocityControl: VelocityPID,
    gyro: HeadingProvider,
    maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
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
            driveMotor = driveMotors.topLeft
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    val topRight = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.topRight,
            turnEncoder = turnEncoders.topRight,
            driveMotor = driveMotors.topRight
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    val bottomLeft = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.bottomLeft,
            turnEncoder = turnEncoders.bottomLeft,
            driveMotor = driveMotors.bottomLeft
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    val bottomRight = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.bottomRight,
            turnEncoder = turnEncoders.bottomRight,
            driveMotor = driveMotors.bottomRight
        ),
        turnControl,
        velocityControl,
        staticVoltageStall
    )

    return EncoderHolonomicDrivetrain(
        topLeft, topRight, bottomLeft, bottomRight,
        gyro, maxModuleSpeed, gearRatio, wheelDiameter,
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
    public val gyro: HeadingProvider,
    public val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    override val gearRatio: Double = DEFAULT_GEAR_RATIO,
    override val wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    public val startingPose: UnitPose2d = UnitPose2d(),
    // do drivetrain.fieldRelativeDrive = false to turn this option off.
    public var fieldRelativeDrive: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier
): SubsystemBase(), RobotPoseSupplier, WheelRatioProvider{

    // not used anywhere else; only used for the setDesiredModuleStates function
    private enum class ControlMode{
        OPEN_LOOP,CLOSED_LOOP
    }
    private var currentControlMode = ControlMode.CLOSED_LOOP

    // Stores the pose suppliers for the drivetrain
    private val allPoseSuppliers: MutableList<RobotPoseSupplier> = poseSuppliers.toMutableList()

    /**
     * adds a variable amount of pose suppliers to the drivetrain.
     * these pose suppliers will fuse their poses into the pose estimator for more accurate measurements.
     */
    public fun addPoseSuppliers(vararg poseSuppliers: RobotPoseSupplier){
        allPoseSuppliers.addAll(poseSuppliers)
    }


    /*
    Encoder-based functions below
     */

    private val moduleArray = a[topLeft,topRight,bottomLeft,bottomRight]
    private fun averageEncoderPosition() =
        moduleArray.map{it.wheelPosition}.average()

    private fun averageEncoderVelocity() =
        moduleArray.map{it.currentVelocity}.average()

    // wheel radius is wheelDiameter / 2.
    private val wheelTravelPerMotorRadian: Distance = gearRatio * (wheelDiameter / 2)
    private val distanceOffset: Distance = averageEncoderPosition() * wheelTravelPerMotorRadian


    public val distanceTraveled: Distance
        get() =
            (averageEncoderPosition() *
                    wheelTravelPerMotorRadian) - distanceOffset
    public val velocity: Velocity
        get() =
            averageEncoderVelocity() *
                    wheelTravelPerMotorRadian




    /*
    Kinematics + pose estimator
    Uses SuperSwerveKinematics, a custom wrapper around WPILib's SwerveDriveKinematics.
     */

    public val kinematics: SuperSwerveKinematics = SuperSwerveKinematics(
        UnitTranslation2d(trackWidth/2,wheelBase/2),
        UnitTranslation2d(trackWidth/2,-wheelBase/2),
        UnitTranslation2d(-trackWidth/2,wheelBase/2),
        UnitTranslation2d(-trackWidth/2,-wheelBase/2)
    )

    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyro.heading.asRotation2d(),
        a[
            topLeft.getModulePosition(gearRatio,wheelDiameter),
            topRight.getModulePosition(gearRatio,wheelDiameter),
            bottomLeft.getModulePosition(gearRatio,wheelDiameter),
            bottomRight.getModulePosition(gearRatio,wheelDiameter)
        ],
        startingPose.inUnit(meters)
    )

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

    public fun resetPose(pose: UnitPose2d, gyroAngle: Angle = gyro.heading){
        poseEstimator.resetPosition(
            gyroAngle.asRotation2d(),
            currentModulePositions.toArray(),
            pose.inUnit(meters)
        )
    }


    /**
     * A getter/setter variable that can either fetch the current speeds
     * of each swerve module,
     * or can set the desired speeds of each swerve module.
     * It is reccomended that the [swerveDrive] and [velocityDrive] functions are used instead.
     */
    public var currentModuleStates: ModuleSpeeds
        get() = ModuleSpeeds(
            topLeftState = topLeft.getModuleState(gearRatio,wheelDiameter),
            topRightState = topRight.getModuleState(gearRatio,wheelDiameter),
            bottomLeftState = bottomLeft.getModuleState(gearRatio,wheelDiameter),
            bottomRightState = bottomRight.getModuleState(gearRatio,wheelDiameter)
        )
        set(ms){
            ms.desaturate(maxModuleSpeed)
            if (currentControlMode == ControlMode.CLOSED_LOOP){
                topLeft.setDirectionalVelocity(ms.topLeftSpeed,ms.topLeftAngle,gearRatio,wheelDiameter)
                topRight.setDirectionalVelocity(ms.topRightSpeed,ms.topRightAngle,gearRatio,wheelDiameter)
                bottomLeft.setDirectionalVelocity(ms.bottomLeftSpeed,ms.bottomLeftAngle,gearRatio,wheelDiameter)
                bottomRight.setDirectionalVelocity(ms.bottomRightSpeed,ms.bottomRightAngle,gearRatio,wheelDiameter)
            }else{
                topLeft.setDirectionalPower((ms.topLeftSpeed/maxModuleSpeed).siValue, ms.topLeftAngle)
                topRight.setDirectionalPower((ms.topRightSpeed/maxModuleSpeed).siValue, ms.topRightAngle)
                bottomLeft.setDirectionalPower((ms.bottomLeftSpeed/maxModuleSpeed).siValue, ms.bottomLeftAngle)
                bottomRight.setDirectionalPower((ms.bottomRightSpeed/maxModuleSpeed).siValue, ms.bottomRightAngle)
            }
        }


    /**
     * Gets the current module positions of each swerve module.
     * Returns a [ModulePositions] object,
     * a wrapper around 4 [SwerveModulePosition] objects with kmeasure support.
     */
    public val currentModulePositions: ModulePositions
        get() = ModulePositions(
            topLeftPosition = topLeft.getModulePosition(gearRatio,wheelDiameter),
            topRightPosition = topRight.getModulePosition(gearRatio,wheelDiameter),
            bottomLeftPosition = bottomLeft.getModulePosition(gearRatio,wheelDiameter),
            bottomRightPosition = bottomRight.getModulePosition(gearRatio,wheelDiameter)
        )


    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    public val maxLinearVelocity: Velocity = kinematics.toChassisSpeeds(
        ModuleSpeeds(
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

        ModuleSpeeds(
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
                speeds,gyro.heading.asRotation2d()
            ).correctForDynamics()
        }
        currentControlMode = ControlMode.OPEN_LOOP
        // currentModuleStates = kinematics.toFirstOrderModuleSpeeds(speeds)
        currentModuleStates = kinematics.toSecondOrderModuleSpeeds(speeds,gyro.heading)
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
                gyro.heading
            ).correctForDynamics()
        }else{
            ChassisSpeeds(
                xVelocity,
                yVelocity,
                rotationVelocity
            ).correctForDynamics()
        }
        currentModuleStates = kinematics.toFirstOrderModuleSpeeds(speeds)
        // currentModuleStates = kinematics.toSecondOrderModuleSpeeds(speeds,gyro.heading)
    }

    /**
     * Drives the drivetrain with a specific speed at a specified angle.
     */
    public fun directionalDrive(speed: Velocity, angle: Angle){
        topLeft.setDirectionalVelocity(speed,angle,gearRatio, wheelDiameter)
        topRight.setDirectionalVelocity(speed,angle,gearRatio,wheelDiameter)
        bottomLeft.setDirectionalVelocity(speed,angle,gearRatio,wheelDiameter)
        bottomRight.setDirectionalVelocity(speed,angle,gearRatio,wheelDiameter)
    }

    /**
     * Drives the drivetrain with a specific speed at a specified angle.
     */
    @JvmName("directionalDriveWithPower")
    public fun directionalDrive(power: Double, angle: Angle){
        topLeft.setDirectionalPower(power,angle)
        topRight.setDirectionalPower(power,angle)
        bottomLeft.setDirectionalPower(power,angle)
        bottomRight.setDirectionalPower(power,angle)
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
        topLeft.setDirectionalPower(0.0,-45.degrees)
        topRight.setDirectionalPower(0.0,45.degrees)
        bottomLeft.setDirectionalPower(0.0,45.degrees)
        bottomRight.setDirectionalPower(0.0,-45.degrees)
    }

    /**
     * Makes the drivetrain rotate in place, with a specific velocity.
     */
    public fun rotateInPlace(speed: Velocity) {
        topLeft.setDirectionalVelocity(speed,45.degrees,gearRatio, wheelDiameter)
        topRight.setDirectionalVelocity(speed,-45.degrees,gearRatio,wheelDiameter)
        bottomLeft.setDirectionalVelocity(speed,-45.degrees,gearRatio,wheelDiameter)
        bottomRight.setDirectionalVelocity(speed,45.degrees,gearRatio,wheelDiameter)
    }

    /**
     * Makes the drivetrain rotate in place, with a specific power.
     */
    @JvmName("rotateWithPower")
    public fun rotateInPlace(power: Double){
        topLeft.setDirectionalPower(power,45.degrees)
        topRight.setDirectionalPower(power,-45.degrees)
        bottomLeft.setDirectionalPower(power,-45.degrees)
        bottomRight.setDirectionalPower(power,45.degrees)
    }


    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        topLeft.updateInputsAndLog("TopLeftSwerveModule")
        topRight.updateInputsAndLog("TopRightSwerveModule")
        bottomLeft.updateInputsAndLog("BottomLeftSwerveModule")
        bottomRight.updateInputsAndLog("BottomRightSwerveModule")


        /*
         * Updates the pose estimator with the current module positions,
         * as well as the gyro's heading.
         */
        poseEstimator.update(
            gyro.heading.asRotation2d(),
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

    }


}