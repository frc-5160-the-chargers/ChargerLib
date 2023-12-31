package frc.chargers.commands.drivetrainCommands.turn

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import kotlin.internal.LowPriorityInOverloadResolution


/**
 * Adds a command to the command builder turning the robot
 * to a specified [angle] (with a specified [precision]) using PID and an external
 * [HeadingProvider].
 *
 * @see Precision
 */
context(CommandBuilder, HeadingProvider)
public fun DifferentialDrivetrain.rotateAction(
    angle: Angle,
    pidConstants: PIDConstants,
    precision: Precision<AngleDimension> = Precision.AllowOvershoot
): Command = runSequentially {
    val targetAngle by getOnceDuringRun{ this@HeadingProvider.heading + angle }
    val controller by getOnceDuringRun{
        SuperPIDController(
            pidConstants,
            { this@HeadingProvider.heading },
            target = targetAngle,
            outputRange = Scalar(-1.0)..Scalar(1.0),
            continuousInputRange = 0.degrees..360.degrees
        )
    }

    fun hasHitTarget(): Boolean = when (precision){
        Precision.AllowOvershoot ->
            // checks that both the heading and target angle have the same sign
            this@HeadingProvider.heading * targetAngle >= 0.degrees &&
            // checks whether the heading has hit or exceeded the target.
            abs(this@HeadingProvider.heading) >= abs(targetAngle)

        is Precision.Within -> this@HeadingProvider.heading in precision.allowableError
    }

    loopUntil(
        condition = ::hasHitTarget,
        this@DifferentialDrivetrain
    ){
        arcadeDrive(0.0, controller.calculateOutput().siValue)
    }
}


/**
 * Adds a command to the command builder turning the robot
 * to a specified [angle] (with a specified [precision]) using PID and the [EncoderDifferentialDrivetrain]'s
 * best available [HeadingProvider].
 *
 * @see Precision
 */
context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.rotateAction(
    angle: Angle,
    pidConstants: PIDConstants = controlScheme.robotRotationPID,
    precision: Precision<AngleDimension> = Precision.AllowOvershoot
): Command = with (gyro ?: this as HeadingProvider) {
    rotateAction(angle,pidConstants,precision)
}

/**
 * Adds a command to the command builder turning a swerve drive robot
 * to a specified [angle] (with a specified [precision]) using PID and the [EncoderHolonomicDrivetrain]'s
 * best available [HeadingProvider].
 *
 * @see Precision
 */
context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.rotateAction(
    angle: Angle,
    pidConstants: PIDConstants = controlData.robotRotationPID,
    precision: Precision<AngleDimension> = Precision.AllowOvershoot
): Command = runSequentially {
    fun getHeading(): Angle = gyro?.heading ?: this@EncoderHolonomicDrivetrain.heading

    val targetAngle by getOnceDuringRun{ getHeading() + angle }
    val controller by getOnceDuringRun{
        SuperPIDController(
            pidConstants,
            { getHeading() },
            target = targetAngle,
            outputRange = Scalar(-1.0)..Scalar(1.0),
            continuousInputRange = 0.degrees..360.degrees
        )
    }

    fun hasHitTarget(): Boolean = when (precision){
        Precision.AllowOvershoot ->
            // checks that both the heading and target angle have the same sign
            getHeading() * targetAngle >= 0.degrees &&
                    // checks whether the heading has hit or exceeded the target.
                    abs(getHeading()) >= abs(targetAngle)

        is Precision.Within -> getHeading() in precision.allowableError
    }

    loopUntil(
        condition = ::hasHitTarget,
        this@EncoderHolonomicDrivetrain
    ){
        arcadeDrive(0.0, controller.calculateOutput().siValue)
    }

}















