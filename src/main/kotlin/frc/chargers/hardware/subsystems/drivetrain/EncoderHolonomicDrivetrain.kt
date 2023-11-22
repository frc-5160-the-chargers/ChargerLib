package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.constants.drivetrain.SwerveConstants
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.posemonitors.SwervePoseMonitor
import frc.chargers.hardware.subsystemutils.swervedrive.module.ModuleIOReal
import frc.chargers.hardware.subsystemutils.swervedrive.module.ModuleIOSim
import frc.chargers.hardware.subsystemutils.swervedrive.module.SwerveModule
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveEncoders
import frc.chargers.hardware.subsystemutils.swervedrive.SecondOrderControlScheme
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.utils.a
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.*
import frc.chargers.wpilibextensions.kinematics.swerve.*
import org.littletonrobotics.junction.Logger


/**
 * A convenience function used to create an [EncoderHolonomicDrivetrain], with [ModuleIOSim] as the Module IO.t
 *
 * In other words, it creates an EncoderHolonomicDrivetrain that can be used in simulation.
 */
public fun simEncoderHolonomicDrivetrain(
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    controlScheme: SwerveControl,
    constants: SwerveConstants,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
): EncoderHolonomicDrivetrain = EncoderHolonomicDrivetrain(
    topLeft = SwerveModule(
        "Drivetrain(Swerve)/TopLeftSwerveModule",
        ModuleIOSim(
            turnGearbox, driveGearbox, constants.turnGearRatio, constants.driveGearRatio, constants.turnInertiaMoment, constants.driveInertiaMoment
        ), controlScheme
    ),
    topRight = SwerveModule(
        "Drivetrain(Swerve)/TopRightSwerveModule",
        ModuleIOSim(
            turnGearbox, driveGearbox, constants.turnGearRatio, constants.driveGearRatio, constants.turnInertiaMoment, constants.driveInertiaMoment
        ), controlScheme
    ),
    bottomLeft = SwerveModule(
        "Drivetrain(Swerve)/BottomLeftSwerveModule",
        ModuleIOSim(
            turnGearbox, driveGearbox, constants.turnGearRatio, constants.driveGearRatio, constants.turnInertiaMoment, constants.driveInertiaMoment
        ), controlScheme
    ),
    bottomRight = SwerveModule(
        "Drivetrain(Swerve)/BottomRightSwerveModule",
        ModuleIOSim(
            turnGearbox, driveGearbox, constants.turnGearRatio, constants.driveGearRatio, constants.turnInertiaMoment, constants.driveInertiaMoment
        ), controlScheme
    ),
    controlScheme, constants, gyro, startingPose, *poseSuppliers
)


/**
 * A Convenience function for creating an [EncoderHolonomicDrivetrain] with [ModuleIOReal] as the swerve module IO.
 *
 * In other words, it creates a swerve drivetrain that can be used on the real robot.
 *
 * It uses the [SwerveMotors] and [SwerveEncoders] classes
 * instead of defining individual swerve modules.
 *
 * This allows the motors and encoders to all be group-configured.
 */
public fun realEncoderHolonomicDrivetrain(
    turnMotors: SwerveMotors,
    turnEncoders: SwerveEncoders,
    driveMotors: SwerveMotors,
    controlScheme: SwerveControl,
    constants: SwerveConstants,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    invertTurnMotors: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier,
): EncoderHolonomicDrivetrain {
    if (invertTurnMotors){
        turnMotors.apply{
            topLeft.inverted = true
            topRight.inverted = true
            bottomLeft.inverted = true
            bottomRight.inverted = true
        }
    }

    val topLeft = SwerveModule(
        "Drivetrain(Swerve)/TopLeftSwerveModule",
        ModuleIOReal(
            turnMotor = turnMotors.topLeft,
            turnEncoder = turnEncoders.topLeft,
            driveMotor = driveMotors.topLeft,
            constants.driveGearRatio, constants.turnGearRatio
        ),
        controlScheme
    )

    val topRight = SwerveModule(
        "Drivetrain(Swerve)/TopRightSwerveModule",
        ModuleIOReal(
            turnMotor = turnMotors.topRight,
            turnEncoder = turnEncoders.topRight,
            driveMotor = driveMotors.topRight,
            constants.driveGearRatio, constants.turnGearRatio
        ),
        controlScheme
    )

    val bottomLeft = SwerveModule(
        "Drivetrain(Swerve)/BottomLeftSwerveModule",
        ModuleIOReal(
            turnMotor = turnMotors.bottomLeft,
            turnEncoder = turnEncoders.bottomLeft,
            driveMotor = driveMotors.bottomLeft,
            constants.driveGearRatio,constants.turnGearRatio
        ),
        controlScheme
    )

    val bottomRight = SwerveModule(
        "Drivetrain(Swerve)/BottomRightSwerveModule",
        ModuleIOReal(
            turnMotor = turnMotors.bottomRight,
            turnEncoder = turnEncoders.bottomRight,
            driveMotor = driveMotors.bottomRight,
            constants.driveGearRatio, constants.turnGearRatio
        ),
        controlScheme
    )

    return EncoderHolonomicDrivetrain(
        topLeft, topRight, bottomLeft, bottomRight,
        controlScheme, constants, gyro, startingPose, *poseSuppliers
    )
}







/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 *
 * Note: TrackWidth is the horizontal length of the robot, while wheelBase is the vertical length of the robot.
 */
public class EncoderHolonomicDrivetrain(
    public val topLeft: SwerveModule,
    public val topRight: SwerveModule,
    public val bottomLeft: SwerveModule,
    public val bottomRight: SwerveModule,
    private val controlScheme: SwerveControl,
    private val constants: SwerveConstants,
    private val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: RobotPoseSupplier,
): SubsystemBase(), ZeroableHeadingProvider{

    init{
        Thread{
            Thread.sleep(1000)
            if (gyro is ZeroableHeadingProvider){
                gyro.zeroHeading()
            }
        }.start()
    }


    /**
     * The pose estimator of the [EncoderHolonomicDrivetrain].
     */
    public val poseEstimator: SwervePoseMonitor = SwervePoseMonitor(
        *poseSuppliers,
        gyro = gyro, startingPose = startingPose
    )


    private var angleOffset = Angle(0.0)
    /**
     * Heading of the robot; from 0-360 degrees.
     */
    override val heading: Angle get() = poseEstimator.heading - angleOffset

    override fun zeroHeading(){
        angleOffset = heading
    }


    /*
    Summary of Kinematics usage:

    the EncoderHolonomicDrivetrain has 4 different control modes,
    represented using the SwerveControl class.

    These control modes control the drivetrain in slightly different ways.

    1. PIDFirstOrder uses basic PID control for swerve angle targeting,
    in addition to basic(first order) kinematics.
    The kinematics converts x speed, y speed, and rotation speed into target angles and speeds for each swerve module.

    2. PIDSecondOrder uses basic PID control and second order kinematics.
    Second order kinematics is similar to first order, except that it does some extra computation
    to reduce drift of the robot when driving while turning.

    3. ProfiledPIDFirstOrder uses Profiled PID control instead of regular PID control. Still in development right now.

    4. ProfiledPIDSecondOrder uses profiled PID control and second order kinematics.

     */


    /**
     * The kinematics class for the drivetrain.
     * Uses [SuperSwerveDriveKinematics], a custom wrapper class around
     * the [edu.wpi.first.math.kinematics.SwerveDriveKinematics] class.
     */
    public val kinematics: SuperSwerveDriveKinematics = SuperSwerveDriveKinematics(
        UnitTranslation2d(constants.trackWidth/2,constants.wheelBase/2),
        UnitTranslation2d(constants.trackWidth/2,-constants.wheelBase/2),
        UnitTranslation2d(-constants.trackWidth/2,constants.wheelBase/2),
        UnitTranslation2d(-constants.trackWidth/2,-constants.wheelBase/2)
    )








    /*
    Control Mode-based functions:
    OPEN_LOOP indicates percent-out(power) based drive,
    while CLOSED_LOOP uses PID and feedforward for velocity-based driving.
     */
    private enum class ControlMode{
        OPEN_LOOP,CLOSED_LOOP
    }
    private var currentControlMode: ControlMode = ControlMode.CLOSED_LOOP









    /*
    Encoder-based functions below. They use similar tactics to the EncoderDifferentialDrivetrain class.
     */
    private val wheelRadius = constants.wheelDiameter / 2.0
    private val moduleArray = a[topLeft,topRight,bottomLeft,bottomRight]
    private fun averageEncoderPosition() = moduleArray.map{it.wheelPosition}.average()
    private fun averageEncoderVelocity() = moduleArray.map{it.currentVelocity}.average()
    
    private val distanceOffset: Distance = averageEncoderPosition() * wheelRadius




    /**
     * The distance the robot has traveled in total.
     */
    public val distanceTraveled: Distance get() = (averageEncoderPosition() * wheelRadius) - distanceOffset

    /**
     * The current overall velocity of the robot.
     */
    public val velocity: Velocity get() = averageEncoderVelocity() * wheelRadius










    /**
     * A getter/setter variable that can either fetch the current speeds
     * of each swerve module,
     * or can set the desired speeds of each swerve module.
     * It is recommended that the [swerveDrive] and [velocityDrive] functions are used instead.
     */
    public var currentModuleStates: ModuleStateGroup
        get() = ModuleStateGroup(
            topLeftState = topLeft.getModuleState(wheelRadius),
            topRightState = topRight.getModuleState(wheelRadius),
            bottomLeftState = bottomLeft.getModuleState(wheelRadius),
            bottomRightState = bottomRight.getModuleState(wheelRadius)
        ).also{
            Logger.getInstance().recordOutput("Drivetrain(Swerve)/CurrentModuleStates",*it.toArray())
        }
        set(ms){
            ms.desaturate(constants.maxModuleSpeed)
            var secondOrderTurnSpeedTL = AngularVelocity(0.0)
            var secondOrderTurnSpeedTR = AngularVelocity(0.0)
            var secondOrderTurnSpeedBL = AngularVelocity(0.0)
            var secondOrderTurnSpeedBR = AngularVelocity(0.0)
            if (controlScheme is SecondOrderControlScheme && ms is SecondOrderModuleStateGroup){
                secondOrderTurnSpeedTL = ms.topLeftTurnSpeed
                secondOrderTurnSpeedTR = ms.topRightTurnSpeed
                secondOrderTurnSpeedBL = ms.bottomLeftTurnSpeed
                secondOrderTurnSpeedBR = ms.bottomRightTurnSpeed
            }
            if (currentControlMode == ControlMode.CLOSED_LOOP){

                topLeft.setDirectionalVelocity(ms.topLeftSpeed / wheelRadius,ms.topLeftAngle,secondOrderTurnSpeedTL)
                topRight.setDirectionalVelocity(ms.topRightSpeed / wheelRadius,ms.topRightAngle,secondOrderTurnSpeedTR)
                bottomLeft.setDirectionalVelocity(ms.bottomLeftSpeed / wheelRadius,ms.bottomLeftAngle,secondOrderTurnSpeedBL)
                bottomRight.setDirectionalVelocity(ms.bottomRightSpeed / wheelRadius,ms.bottomRightAngle,secondOrderTurnSpeedBR)
            }else{
                topLeft.setDirectionalPower((ms.topLeftSpeed/constants.maxModuleSpeed).siValue, ms.topLeftAngle,secondOrderTurnSpeedTL)
                topRight.setDirectionalPower((ms.topRightSpeed/constants.maxModuleSpeed).siValue, ms.topRightAngle,secondOrderTurnSpeedTR)
                bottomLeft.setDirectionalPower((ms.bottomLeftSpeed/constants.maxModuleSpeed).siValue, ms.bottomLeftAngle,secondOrderTurnSpeedBL)
                bottomRight.setDirectionalPower((ms.bottomRightSpeed/constants.maxModuleSpeed).siValue, ms.bottomRightAngle,secondOrderTurnSpeedBR)
            }
            Logger.getInstance().recordOutput("Drivetrain(Swerve)/DesiredModuleStates", ms.topLeftState,ms.topRightState,ms.bottomLeftState,ms.bottomRightState)

        }


    /**
     * Gets the current module positions of each swerve module.
     * Returns a [ModulePositionGroup] object,
     * a wrapper around 4 [SwerveModulePosition] objects with units support.
     */
    public val currentModulePositions: ModulePositionGroup
        get() = ModulePositionGroup(
            topLeftDistance = topLeft.wheelPosition.inUnit(radians) * wheelRadius,
            topRightDistance = topRight.wheelPosition.inUnit(radians) * wheelRadius,
            bottomLeftDistance = bottomLeft.wheelPosition.inUnit(radians) * wheelRadius,
            bottomRightDistance = bottomRight.wheelPosition.inUnit(radians) * wheelRadius,
            topLeftAngle = topLeft.currentDirection,
            topRightAngle = topRight.currentDirection,
            bottomLeftAngle = bottomLeft.currentDirection,
            bottomRightAngle = bottomRight.currentDirection
        ).also{
            Logger.getInstance().apply{
                recordOutput("ModulePositions/topLeft/DistanceMeters", it.topLeftDistance.inUnit(meters))
                recordOutput("ModulePositions/topRight/DistanceMeters", it.topRightDistance.inUnit(meters))
                recordOutput("ModulePositions/bottomLeft/DistanceMeters", it.bottomLeftDistance.inUnit(meters))
                recordOutput("ModulePositions/bottomRight/DistanceMeters", it.bottomRightDistance.inUnit(meters))

                recordOutput("ModulePositions/topLeft/AngleDeg",it.topLeftAngle.inUnit(degrees))
                recordOutput("ModulePositions/topRight/AngleDeg",it.topRightAngle.inUnit(degrees))
                recordOutput("ModulePositions/bottomLeft/AngleDeg",it.bottomLeftAngle.inUnit(degrees))
                recordOutput("ModulePositions/bottomRight/AngleDeg",it.bottomRightAngle.inUnit(degrees))
            }
        }









    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    public val maxLinearVelocity: Velocity = abs(kinematics.toChassisSpeeds(
        ModuleStateGroup(
            topLeftSpeed = constants.maxModuleSpeed,
            topRightSpeed = constants.maxModuleSpeed,
            bottomLeftSpeed = constants.maxModuleSpeed,
            bottomRightSpeed = constants.maxModuleSpeed,
            topLeftAngle = Angle(0.0),
            topRightAngle = Angle(0.0),
            bottomLeftAngle = Angle(0.0),
            bottomRightAngle = Angle(0.0)
        )
    ).xVelocity)

    /**
     * The max angular velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential(with each being oriented at a 45 or -45 degrees angle),
     * then calculating the output using the kinematics object.
     */
    public val maxRotationalVelocity: AngularVelocity = abs(kinematics.toChassisSpeeds(
        ModuleStateGroup(
            topLeftSpeed = constants.maxModuleSpeed,
            topRightSpeed = -constants.maxModuleSpeed,
            bottomLeftSpeed = constants.maxModuleSpeed,
            bottomRightSpeed = -constants.maxModuleSpeed,
            topLeftAngle = -45.degrees,
            topRightAngle = 45.degrees,
            bottomLeftAngle = 45.degrees,
            bottomRightAngle = -45.degrees
        )
    ).rotationSpeed)






    // used for open-loop drive; second kinematics
    private var previousChassisPowers = ChassisPowers()
    private var previousHeading = 0.0.degrees


    private val mostReliableHeading: Angle
        get() = (gyro?.heading ?: this.heading).inputModulus(0.0.degrees..360.degrees)

    /* Below are the open-loop drive functions.
    */

    /**
     * A generic drive function; mainly used if driving at a specific velocity is not required, or during teleop.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun swerveDrive(
        xPower: Double,
        yPower: Double,
        rotationPower: Double,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ): Unit = swerveDrive(ChassisPowers(xPower,yPower,rotationPower), fieldRelative)

    /**
     * A generic drive function; mainly used if driving at a specific velocity is not required, or during teleop.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun swerveDrive(
        powers: ChassisPowers,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ){
        val speeds = powers.toChassisSpeeds(maxLinearVelocity,maxRotationalVelocity)
        currentControlMode = ControlMode.OPEN_LOOP


        /*
        4481's second order swerve kinematics already applies field relative drive;
        thus, fromFieldRelativeSpeeds is not called if second order kinematics is used.

        Note: the multipliers here are relatively arbitrary; they are simply what has worked best for us.
        In addition, second order kinematics in open-loop control over-corrects
         */
        if (controlScheme is SecondOrderControlScheme){
            currentModuleStates = kinematics.toSecondOrderModuleStateGroup(
                speeds,
                mostReliableHeading,
                fieldRelative
            )
        }else{

            // sets module states using the ModuleStateGroup class
            currentModuleStates = kinematics.toFirstOrderModuleStateGroup(
                if(fieldRelative){
                    ChassisSpeeds.fromFieldRelativeSpeeds(speeds, mostReliableHeading.asRotation2d())
                }else{
                    speeds
                }.correctForDynamics(driftRate = if (RobotBase.isReal()) 2.2 else 1.8)
            )
        }

        currentControlMode = ControlMode.CLOSED_LOOP
        previousChassisPowers = powers
        previousHeading = mostReliableHeading
    }




    /* Here are the closed-loop drive functions. */


    /**
     * Drives the robot with specific speeds; uses closed loop control.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun velocityDrive(
        xVelocity: Velocity,
        yVelocity: Velocity,
        rotationVelocity: AngularVelocity,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ): Unit = velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationVelocity), fieldRelative)


    /**
     * Drives the robot with specific speeds; uses closed loop control.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun velocityDrive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ){

        currentControlMode = ControlMode.CLOSED_LOOP

        /*
        4481's second order swerve kinematics already applies field relative drive;
        thus, fromFieldRelativeSpeeds is not called if second order kinematics is used.
         */
        currentModuleStates = if(controlScheme is SecondOrderControlScheme){
            kinematics.toSecondOrderModuleStateGroup(
                speeds,
                mostReliableHeading,
                fieldRelative
            )
        }else{
            kinematics.toFirstOrderModuleStateGroup(
                if(fieldRelative){
                    ChassisSpeeds.fromFieldRelativeSpeeds(speeds, mostReliableHeading.asRotation2d())
                }else{
                    speeds
                }.correctForDynamics(driftRate = if (RobotBase.isReal()) 1.3 else 1.0)
            )
        }

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
        topLeft.updateAndProcessInputs()
        topRight.updateAndProcessInputs()
        bottomLeft.updateAndProcessInputs()
        bottomRight.updateAndProcessInputs()

        Logger.getInstance().recordOutput("Drivetrain(Swerve)/CurrentModuleStates", *currentModuleStates.toArray())
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/OverallVelocityMetersPerSec",velocity.inUnit(meters/seconds))
    }


}