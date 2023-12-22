package frc.chargers.commands.drivetrainCommands.turn

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.DifferentialDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import kotlin.internal.LowPriorityInOverloadResolution


/**
 * Adds a command to the command builder turning the robot
 * to a specified [angle] (with a specified [precision]) using PID.
 *
 * @see Precision
 */
context(CommandBuilder, HeadingProvider)
@LowPriorityInOverloadResolution
public fun DifferentialDrivetrain.rotateAction(
    angle: Angle,
    pidConstants: PIDConstants = PIDConstants(0.6,0.0,0.0),
    precision: Precision<AngleDimension> = Precision.AllowOvershoot
): Command = runSequentially {
    val targetAngle by getOnceDuringRun{ this@HeadingProvider.heading + angle }
    val controller by getOnceDuringRun{
        UnitSuperPIDController(
            pidConstants,
            { this@HeadingProvider.heading },
            Scalar(-1.0)..Scalar(1.0),
            target = targetAngle,
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











