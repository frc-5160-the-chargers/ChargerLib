package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.swerve.SwerveMotors
import frc.chargers.hardware.swerve.SwerveEncoders
import frc.chargers.hardware.swerve.SecondOrderControlScheme
import frc.chargers.hardware.swerve.SwerveControl
import frc.chargers.hardware.swerve.module.*
import frc.chargers.utils.a
import frc.chargers.utils.math.units.Inertia
import frc.chargers.utils.p
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.*
import frc.chargers.wpilibextensions.kinematics.swerve.*
import org.littletonrobotics.junction.Logger
import kotlin.internal.LowPriorityInOverloadResolution


@PublishedApi
internal val DEFAULT_MAX_MODULE_SPEED: Velocity = 4.5.ofUnit(meters/seconds)


/**
 * A convenience function used to create an [EncoderHolonomicDrivetrain], with [ModuleIOSim] as the Module IO.
 */
public fun simEncoderHolonomicDrivetrain(
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    controlScheme: SwerveControl,
    maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    loopPeriod: Time = 20.milli.seconds,
): EncoderHolonomicDrivetrain = EncoderHolonomicDrivetrain(
    topLeft = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), controlScheme
    ),
    topRight = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), controlScheme
    ),
    bottomLeft = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), controlScheme
    ),
    bottomRight = SwerveModule(
        ModuleIOSim(
            turnGearbox, driveGearbox, loopPeriod, turnGearRatio, driveGearRatio, turnInertiaMoment, driveInertiaMoment
        ), controlScheme
    ),
    controlScheme, maxModuleSpeed, wheelDiameter, trackWidth, wheelBase
)


/**
 * A Convenience function for creating an [EncoderHolonomicDrivetrain] with [ModuleIOReal] as the swerve module IO.
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
    maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    loopPeriod: Time = 20.milli.seconds,
): EncoderHolonomicDrivetrain{
    val topLeft = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.topLeft,
            turnEncoder = turnEncoders.topLeft,
            driveMotor = driveMotors.topLeft,
            driveGearRatio, turnGearRatio
        ),
        controlScheme
    )

    val topRight = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.topRight,
            turnEncoder = turnEncoders.topRight,
            driveMotor = driveMotors.topRight,
            driveGearRatio, turnGearRatio
        ),
        controlScheme
    )

    val bottomLeft = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.bottomLeft,
            turnEncoder = turnEncoders.bottomLeft,
            driveMotor = driveMotors.bottomLeft,
            driveGearRatio,turnGearRatio
        ),
        controlScheme
    )

    val bottomRight = SwerveModule(
        ModuleIOReal(
            turnMotor = turnMotors.bottomRight,
            turnEncoder = turnEncoders.bottomRight,
            driveMotor = driveMotors.bottomRight,
            driveGearRatio, turnGearRatio
        ),
        controlScheme
    )

    return EncoderHolonomicDrivetrain(
        topLeft, topRight, bottomLeft, bottomRight,
        controlScheme, maxModuleSpeed, wheelDiameter,
        trackWidth, wheelBase, loopPeriod
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
    public val controlScheme: SwerveControl,
    private val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    private val loopPeriod: Time = 0.02.seconds
): SubsystemBase(){

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
    Control Mode-based functions:
    OPEN_LOOP indicates percent-out(power) based drive,
    while CLOSED_LOOP uses PID and feedforward for velocity-based driving.
     */
    private enum class ControlMode{
        OPEN_LOOP,CLOSED_LOOP
    }
    private var currentControlMode = ControlMode.CLOSED_LOOP






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
        ).also{
            Logger.getInstance().recordOutput("Drivetrain(Swerve)/CurrentModuleStates",*it.toArray())
        }
        set(ms){
            ms.desaturate(maxModuleSpeed)
            var topLeftV = 0.0.volts
            var topRightV = 0.0.volts
            var bottomLeftV = 0.0.volts
            var bottomRightV = 0.0.volts
            if (controlScheme is SecondOrderControlScheme && ms is SecondOrderModuleStateGroup){
                topLeftV = controlScheme.turnFF.calculate(ms.topLeftTurnSpeed)
                topRightV = controlScheme.turnFF.calculate(ms.topRightTurnSpeed)
                bottomLeftV = controlScheme.turnFF.calculate(ms.bottomLeftTurnSpeed)
                bottomRightV = controlScheme.turnFF.calculate(ms.bottomRightTurnSpeed)

                Logger.getInstance().recordOutput(
                    "Drivetrain(Swerve)/SecondOrderTurnSpeedsRadPerSec",
                    p[
                        ms.topLeftTurnSpeed.inUnit(radians/seconds),
                        ms.topRightTurnSpeed.inUnit(radians/seconds),
                        ms.bottomLeftTurnSpeed.inUnit(radians/seconds),
                        ms.bottomRightTurnSpeed.inUnit(radians/seconds),
                    ]
                )

                Logger.getInstance().recordOutput(
                    "Drivetrain(Swerve)/SecondOrderTurnAppliedVolts",
                    p[
                        topLeftV.inUnit(volts),
                        topRightV.inUnit(volts),
                        bottomLeftV.inUnit(volts),
                        bottomRightV.inUnit(volts)
                    ]
                )

            }
            if (currentControlMode == ControlMode.CLOSED_LOOP){

                topLeft.setDirectionalVelocity(ms.topLeftSpeed / wheelRadius,ms.topLeftAngle,topLeftV)
                topRight.setDirectionalVelocity(ms.topRightSpeed / wheelRadius,ms.topRightAngle,topRightV)
                bottomLeft.setDirectionalVelocity(ms.bottomLeftSpeed / wheelRadius,ms.bottomLeftAngle,bottomLeftV)
                bottomRight.setDirectionalVelocity(ms.bottomRightSpeed / wheelRadius,ms.bottomRightAngle,bottomRightV)
            }else{
                topLeft.setDirectionalPower((ms.topLeftSpeed/maxModuleSpeed).siValue, ms.topLeftAngle,topLeftV)
                topRight.setDirectionalPower((ms.topRightSpeed/maxModuleSpeed).siValue, ms.topRightAngle,topRightV)
                bottomLeft.setDirectionalPower((ms.bottomLeftSpeed/maxModuleSpeed).siValue, ms.bottomLeftAngle,bottomLeftV)
                bottomRight.setDirectionalPower((ms.bottomRightSpeed/maxModuleSpeed).siValue, ms.bottomRightAngle,bottomRightV)
            }
            Logger.getInstance().recordOutput("Drivetrain(Swerve)/DesiredModuleStates", ms.topLeftState,ms.topRightState,ms.bottomLeftState,ms.bottomRightState)

        }


    /**
     * Gets the current module positions of each swerve module.
     * Returns a [ModulePositionGroup] object,
     * a wrapper around 4 [SwerveModulePosition] objects with kmeasure support.
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
            topLeftSpeed = maxModuleSpeed,
            topRightSpeed = maxModuleSpeed,
            bottomLeftSpeed = maxModuleSpeed,
            bottomRightSpeed = maxModuleSpeed,
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
            topLeftSpeed = maxModuleSpeed,
            topRightSpeed = -maxModuleSpeed,
            bottomLeftSpeed = maxModuleSpeed,
            bottomRightSpeed = -maxModuleSpeed,
            topLeftAngle = -45.degrees,
            topRightAngle = 45.degrees,
            bottomLeftAngle = 45.degrees,
            bottomRightAngle = -45.degrees
        )
    ).rotationSpeed)




    private val isReal by lazy{ RobotBase.isReal()}
    /*
    Adds different multipliers for CorrectForDynamics, depending on if the robot is sim or if it is real.

    Note: numbers were based off of empirical observation.
     */
    private fun ChassisSpeeds.correctForDynamicsOptimized(): ChassisSpeeds =
        correctForDynamics(
            loopPeriod,
            multiplier = if(isReal) 3.0 else 0.7
        )




    /*
    Below are the open-loop drive functions.
     */

    /**
     * non field-oriented.
     */
    @LowPriorityInOverloadResolution
    public fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double): Unit =
        swerveDrive(ChassisPowers(xPower,yPower,rotationPower))

    /**
     * non field-oriented.
     */
    @LowPriorityInOverloadResolution
    public fun swerveDrive(powers: ChassisPowers){
        val speeds = powers.toChassisSpeeds(maxLinearVelocity,maxRotationalVelocity)
        currentControlMode = ControlMode.OPEN_LOOP
        currentModuleStates = kinematics.toFirstOrderModuleStateGroup(speeds.correctForDynamicsOptimized())
        currentControlMode = ControlMode.CLOSED_LOOP
    }

    context(HeadingProvider)
    @JvmName("SwerveDriveFieldOriented")
    public fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double): Unit =
        swerveDrive(ChassisPowers(xPower,yPower,rotationPower))

    context(HeadingProvider)
    @JvmName("SwerveDriveFieldOriented")
    public fun swerveDrive(powers: ChassisPowers){
        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            powers.toChassisSpeeds(maxLinearVelocity,maxRotationalVelocity),
            heading.asRotation2d()
        )
        currentControlMode = ControlMode.OPEN_LOOP

        currentModuleStates = if (controlScheme is SecondOrderControlScheme){
            kinematics.toSecondOrderModuleStateGroup(
                speeds.correctForDynamicsOptimized(),
                heading
            )
        }else{
            kinematics.toFirstOrderModuleStateGroup(speeds.correctForDynamicsOptimized())
        }
        currentControlMode = ControlMode.CLOSED_LOOP
    }









    /*
    Here are the closed-loop drive functions.

    In order to perform field-oriented drive, the context of a HeadingProvider must be given, I.E:

    with(navX as HeadingProvider){
        drivetrain.velocityDrive(ChassisSpeeds(0.1,0.0,0.0))
    }

    If no context is given, the drivetrain will be robot-oriented instead.
     */

    /**
     * non field-oriented.
     */
    @LowPriorityInOverloadResolution
    public fun velocityDrive(speeds: ChassisSpeeds){
        currentControlMode = ControlMode.CLOSED_LOOP
        currentModuleStates = kinematics.toFirstOrderModuleStateGroup(speeds.correctForDynamicsOptimized())
    }

    /**
     * non field-oriented.
     */
    @LowPriorityInOverloadResolution
    public fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationVelocity: AngularVelocity): Unit =
        velocityDrive(
            ChassisSpeeds(xVelocity,yVelocity,rotationVelocity)
        )

    context(HeadingProvider)
    @JvmName("VelocityDriveFieldOriented")
    public fun velocityDrive(speeds: ChassisSpeeds){
        val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, heading.asRotation2d()
        )
        currentControlMode = ControlMode.CLOSED_LOOP

        currentModuleStates = if(controlScheme is SecondOrderControlScheme){
            kinematics.toSecondOrderModuleStateGroup(newSpeeds.correctForDynamicsOptimized(),heading)
        }else{
            kinematics.toFirstOrderModuleStateGroup(newSpeeds.correctForDynamicsOptimized())
        }
    }

    context(HeadingProvider)
    @JvmName("VelocityDriveFieldOriented")
    public fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationVelocity: AngularVelocity): Unit =
        velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationVelocity))











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

        Logger.getInstance().recordOutput("Drivetrain(Swerve)/CurrentModuleStates", *currentModuleStates.toArray())
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/OverallVelocityMetersPerSec",velocity.inUnit(meters/seconds))
    }


}