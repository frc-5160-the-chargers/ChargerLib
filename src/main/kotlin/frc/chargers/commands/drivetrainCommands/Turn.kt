package frc.chargers.commands.drivetrainCommands

import com.batterystaple.kmeasure.*
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.CodeBlockContext
import frc.chargers.commands.CommandBuilder
import frc.chargers.constants.TurnPIDConstants
import frc.chargers.hardware.sensors.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.DifferentialDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.utils.pid.PIDConstants
import frc.chargers.utils.pid.UnitSuperPIDController
import kotlin.internal.LowPriorityInOverloadResolution

/**
 * Adds a command to the command builder turning the robot
 * at a specified [rotationPower] until reaching a specified [angle].
 */
context(CommandBuilder, HeadingProvider)
@LowPriorityInOverloadResolution
public fun DifferentialDrivetrain.turn(angle: Angle, rotationPower: Double): Command {
    val startHeading = heading
    return runUntil(
        when {
            angle < 0.degrees -> { { heading < startHeading + angle } }
            angle > 0.degrees -> { { heading > startHeading + angle } }
            else -> { { true } }
        },
        this
    ) {
        arcadeDrive(power = 0.0, rotation = rotationPower)
    }
}

/**
 * Adds a command to the command builder turning the robot
 * to a specified [angle] (with a specified [precision]) using PID,
 * with PID constants gotten from the local context.
 *
 * @see Precision
 */
context(CommandBuilder, HeadingProvider, TurnPIDConstants)
public fun DifferentialDrivetrain.turn(angle: Angle, precision: Precision<AngleDimension> = Precision.AllowOvershoot): Command =
    turn(angle, precision, turnPIDConstants)

/**
 * Adds a command to the command builder turning the robot
 * to a specified [angle] (with a specified [precision]) using PID.
 *
 * @see Precision
 */
context(CommandBuilder, HeadingProvider)
@LowPriorityInOverloadResolution
public fun DifferentialDrivetrain.turn(angle: Angle, precision: Precision<AngleDimension> = Precision.AllowOvershoot, pidConstants: PIDConstants): Command {
    val targetHeading = heading
    val pidController = UnitSuperPIDController(
        pidConstants = pidConstants,
        getInput = { heading },
        outputRange = Scalar(0.0)..Scalar(1.0),
        target = targetHeading
    )

    val runToTarget: CodeBlockContext.() -> Unit = { arcadeDrive(power = 0.0, rotation = pidController.calculateOutput().siValue) }

    when(precision) {
        Precision.AllowOvershoot -> {
            return runUntil(
                when {
                    angle < 0.degrees -> { { heading < targetHeading } }
                    angle > 0.degrees -> { { heading > targetHeading } }
                    else -> { { true } } // If target rotation is 0 degrees or NaN, immediately stop
                },
                this,
                execute = runToTarget
            )
        }
        is Precision.Within -> {
            return runUntil({ pidController.error in precision.allowableError }, this, execute = runToTarget)
        }
    }
}


context(CodeBlockContext, CommandBuilder)
@Suppress("unused", "DeprecatedCallableAddReplaceWith", "UNUSED_PARAMETER")
@Deprecated("Can't create a command inside a CodeBlock", level = DeprecationLevel.ERROR)
public fun DifferentialDrivetrain.turn(angle: Angle, rotationPower: Double): Command = error("Cannot call this from inside a CodeBlock - make sure you're inside a CommandBuilder")

context(CodeBlockContext, CommandBuilder)
@Suppress("unused", "DeprecatedCallableAddReplaceWith", "UNUSED_PARAMETER")
@Deprecated("Can't create a command inside a CodeBlock", level = DeprecationLevel.ERROR)
public fun DifferentialDrivetrain.turn(angle: Angle, precision: Precision<AngleDimension>, pidConstants: PIDConstants): Command = error("Cannot call this from inside a CodeBlock - make sure you're inside a CommandBuilder")