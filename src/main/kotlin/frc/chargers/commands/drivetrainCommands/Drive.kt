package frc.chargers.commands.drivetrainCommands

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.CodeBlockContext
import frc.chargers.commands.CommandBuilder
import frc.chargers.constants.TurnPIDConstants
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.DifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import kotlin.internal.LowPriorityInOverloadResolution

private const val DEFAULT_MAX_STEERING_POWER = 0.3

private val DEFAULT_MAX_STEERING_VELOCITY = AngularVelocity(0.3)

/**
 * Adds a command to the command builder driving the robot for
 * a specified [time] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder, HeadingProvider)
@JvmName("driveTime")
@LowPriorityInOverloadResolution
public fun DifferentialDrivetrain.driveStraight(time: Time, power: Double, steeringPIDConstants: PIDConstants, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = runSequentially {
    val initialHeading by getOnceDuringRun { this@HeadingProvider.heading }

    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { this@HeadingProvider.heading },
            target = initialHeading,
            outputRange = Scalar(0.0)..Scalar(maxSteeringPower)
        )
    }

    loopFor(time, this@driveStraight) { arcadeDrive(power, pid.calculateOutput().value) }
}


/**
 * Adds a command to the command builder driving the robot
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation and
 * getting PID constants from the local context.
 */
context(CommandBuilder, HeadingProvider, TurnPIDConstants)
@JvmName("driveDistance")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, power: Double, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = with(this@HeadingProvider) {
    driveStraight(distance, power, turnPIDConstants, maxSteeringPower)
}

/**
 * Adds a command to the command builder driving the robot
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder, HeadingProvider)
@JvmName("driveDistance")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, power: Double, steeringPIDConstants: PIDConstants, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = runSequentially {
    val initialPosition by getOnceDuringRun { distanceTraveled }
    val initialHeading by getOnceDuringRun { this@HeadingProvider.heading }
    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { this@HeadingProvider.heading },
            target = initialHeading,
            outputRange = Scalar(0.0)..Scalar(maxSteeringPower)
        )
    }

    loopUntil(
        if (power > 0) {
            { (distanceTraveled - initialPosition) >= distance }
        } else {
            { (distanceTraveled - initialPosition) <= distance }
        },
        this@driveStraight
    ) {
        arcadeDrive(power, pid.calculateOutput().value)
    }
}

context(CommandBuilder, TurnPIDConstants)
@JvmName("driveDistanceWithVelocity")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, velocity: Velocity, maxSteeringVelocity: AngularVelocity = DEFAULT_MAX_STEERING_VELOCITY): Command =
    driveStraight(distance, velocity, turnPIDConstants, maxSteeringVelocity)

context(CommandBuilder)
@JvmName("driveDistanceWithVelocity")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, velocity: Velocity, steeringPIDConstants: PIDConstants, maxSteeringVelocity: AngularVelocity = DEFAULT_MAX_STEERING_VELOCITY): Command = runSequentially {
    val initialPosition by getOnceDuringRun { distanceTraveled }
    val initialHeading by getOnceDuringRun { heading }
    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { heading },
            target = initialHeading,
            outputRange = -maxSteeringVelocity..maxSteeringVelocity
        )
    }

    loopUntil(
        if (velocity > Velocity(0.0)) {
            { (distanceTraveled - initialPosition) >= distance }
        } else {
            { (distanceTraveled - initialPosition) <= distance }
        },
        this@driveStraight
    ) {
        velocityDrive(ChassisSpeeds(velocity,Quantity(0.0),pid.calculateOutput()))
    }
}

context(CommandBuilder, HeadingProvider)
@JvmName("driveTimeWithVelocity")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(time: Time, velocity: Velocity, steeringPIDConstants: PIDConstants, maxSteeringVelocity: AngularVelocity = DEFAULT_MAX_STEERING_VELOCITY): Command = runSequentially {
    val initialHeading by getOnceDuringRun { this@HeadingProvider.heading }

    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { this@HeadingProvider.heading },
            target = initialHeading,
            outputRange = -maxSteeringVelocity..maxSteeringVelocity
        )
    }


    loopFor(time, this@driveStraight) { velocityDrive(ChassisSpeeds(velocity,Quantity(0.0),pid.calculateOutput())) }
}

/*
context(CommandBuilder)
@JvmName("swerveDriveTime")
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.driveStraight(direction: Angle = gyro.heading, time: Time, power: Double): Command = loopFor(time,this){
    directionalDrive(power,direction)
}

context(CommandBuilder)
@JvmName("swerveDriveDistance")
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.driveStraight(direction: Angle = gyro.heading, distance: Distance, power: Double): Command = loopWhile({distanceTraveled < distance},this){
    directionalDrive(power,direction)
}
 */





context(CodeBlockContext, CommandBuilder)
@Suppress("unused", "DeprecatedCallableAddReplaceWith", "UNUSED_PARAMETER", "UnusedReceiverParameter")
@Deprecated("Can't create a command inside a CodeBlock", level = DeprecationLevel.ERROR)
public fun DifferentialDrivetrain.driveStraight(time: Time, power: Double, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = error("Cannot call this from inside a CodeBlock - make sure you're inside a CommandBuilder")

context(CodeBlockContext, CommandBuilder)
@Suppress("unused", "DeprecatedCallableAddReplaceWith", "UNUSED_PARAMETER", "UnusedReceiverParameter")
@Deprecated("Can't create a command inside a CodeBlock", level = DeprecationLevel.ERROR)
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, power: Double, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = error("Cannot call this from inside a CodeBlock - make sure you're inside a CommandBuilder")
