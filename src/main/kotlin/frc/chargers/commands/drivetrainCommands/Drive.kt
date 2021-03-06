package frc.chargers.commands.drivetrainCommands

import com.batterystaple.kmeasure.Distance
import com.batterystaple.kmeasure.Scalar
import com.batterystaple.kmeasure.Time
import com.batterystaple.kmeasure.value
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.CodeBlockContext
import frc.chargers.commands.CommandBuilder
import frc.chargers.constants.TurnPIDConstants
import frc.chargers.hardware.sensors.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.DifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.utils.pid.PIDConstants
import frc.chargers.utils.pid.UnitSuperPIDController
import kotlin.internal.LowPriorityInOverloadResolution

private const val DEFAULT_MAX_STEERING_POWER = 0.3

/**
 * Adds a command to the command builder driving the robot for
 * a specified [time] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder, HeadingProvider)
@JvmName("driveTime")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(time: Time, power: Double, steeringPIDConstants: PIDConstants, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = runSequentially {
    val initialHeading by getOnceDuringRun { this@HeadingProvider.heading }
    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { this@HeadingProvider.heading },
            target = initialHeading,
            outputRange = Scalar(0.0)..Scalar(maxSteeringPower)
        )
    }

    runFor(time, this@driveStraight) { arcadeDrive(power, pid.calculateOutput().value) }
}

/**
 * Adds a command to the command builder driving the robot
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation and
 * getting PID constants from the local context.
 */
context(CommandBuilder, TurnPIDConstants)
@JvmName("driveDistance")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, power: Double, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command =
    driveStraight(distance, power, turnPIDConstants, maxSteeringPower)

/**
 * Adds a command to the command builder driving the robot
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder)
@JvmName("driveDistance")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, power: Double, steeringPIDConstants: PIDConstants, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = runSequentially {
    val initialPosition by getOnceDuringRun { distanceTraveled }
    val initialHeading by getOnceDuringRun { heading }
    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { heading },
            target = initialHeading,
            outputRange = Scalar(0.0)..Scalar(maxSteeringPower)
        )
    }

    runUntil(
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

context(CodeBlockContext, CommandBuilder)
@Suppress("unused", "DeprecatedCallableAddReplaceWith", "UNUSED_PARAMETER")
@Deprecated("Can't create a command inside a CodeBlock", level = DeprecationLevel.ERROR)
public fun DifferentialDrivetrain.driveStraight(time: Time, power: Double, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = error("Cannot call this from inside a CodeBlock - make sure you're inside a CommandBuilder")

context(CodeBlockContext, CommandBuilder)
@Suppress("unused", "DeprecatedCallableAddReplaceWith", "UNUSED_PARAMETER")
@Deprecated("Can't create a command inside a CodeBlock", level = DeprecationLevel.ERROR)
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, power: Double, maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER): Command = error("Cannot call this from inside a CodeBlock - make sure you're inside a CommandBuilder")
