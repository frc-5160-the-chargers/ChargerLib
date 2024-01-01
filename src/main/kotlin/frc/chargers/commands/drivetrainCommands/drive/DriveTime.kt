package frc.chargers.commands.drivetrainCommands.drive

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.commands.commandbuilder.CommandBuilderMarker
import frc.chargers.constants.drivetrain.DEFAULT_MAX_STEERING_POWER
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDrivetrain
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import kotlin.internal.LowPriorityInOverloadResolution


/**
 * Adds a command to the command builder driving the robot for
 * a specified [time] with a specified [power].
 */
context(CommandBuilder)
@CommandBuilderMarker
public fun DifferentialDrivetrain.driveStraightAction(
    time: Time,
    power: Double
): Command = runSequentially{
    loopFor(time, this@DifferentialDrivetrain){
        arcadeDrive(power)
    }

    runOnce(this@DifferentialDrivetrain) {
        stop()
    }
}

/**
 * Adds a command to the command builder driving a swerve-drive robot for
 * a specified [time] with a specified [power], at a certain [direction].
 */
context(CommandBuilder)
@JvmName("driveTime")
public fun EncoderHolonomicDrivetrain.driveStraightAction(
    time: Time,
    power: Double,
    direction: Angle = Angle(0.0),
    fieldRelative: Boolean = false
): Command = runSequentially{

    loopFor(time,this@EncoderHolonomicDrivetrain){
        if (fieldRelative){
            val fieldRelativeDelta = gyro?.heading ?: this@EncoderHolonomicDrivetrain.heading
            topLeft.setDirectionalPower(power,direction - fieldRelativeDelta )
            topRight.setDirectionalPower(power,direction - fieldRelativeDelta )
            bottomLeft.setDirectionalPower(power,direction - fieldRelativeDelta )
            bottomRight.setDirectionalPower(power,direction - fieldRelativeDelta )
        }else{
            topLeft.setDirectionalPower(power,direction)
            topRight.setDirectionalPower(power,direction)
            bottomLeft.setDirectionalPower(power,direction)
            bottomRight.setDirectionalPower(power,direction)
        }
    }

    runOnce(this@EncoderHolonomicDrivetrain){
        stop()
    }

}



/**
 * Adds a command to the command builder driving the robot for
 * a specified [time] with a specified [power], using
 * PID and an external [HeadingProvider] to ensure a straight drive without deviation.
 *
 * This command-returning function requires the context of a [HeadingProvider]. For instance:
 *
 * ```
 * buildCommand{
 *      with(NavX()){
 *          drivetrain.driveStraight(1.seconds, 0.5, PIDConstants(0.1,0.0,0.0))
 *      }
 * }
 */
context(CommandBuilder, HeadingProvider)
@JvmName("driveTime") // ensures that there isn't any platform declaration crashes(Time is represented as a Double during runtime)
public fun DifferentialDrivetrain.driveStraightAction(
    time: Time,
    power: Double,
    steeringPIDConstants: PIDConstants,
    direction: Angle = heading,
    maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER
): Command = runSequentially {
    val initialHeading by getOnceDuringRun { direction }

    val pid by getOnceDuringRun {
        SuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { this@HeadingProvider.heading },
            target = initialHeading,
            outputRange = Scalar(0.0)..Scalar(maxSteeringPower)
        )
    }

    loopFor(time, this@DifferentialDrivetrain) { arcadeDrive(power, pid.calculateOutput().value) }

    runOnce(this@DifferentialDrivetrain){
        stop()
    }
}


/**
 * Adds a command to the command builder driving the robot for
 * a specified [time] with a specified [power], using
 * PID and calculated heading to ensure a straight drive without deviation.
 *
 */
context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraightAction(
    time: Time,
    power: Double,
    steeringPIDConstants: PIDConstants,
    direction: Angle = heading,
    maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER
): Command = with (gyro ?: this){// provides this block with the context of a HeadingProvider
    driveStraightAction(time, power, steeringPIDConstants,direction, maxSteeringPower)
}

