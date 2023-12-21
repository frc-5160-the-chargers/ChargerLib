package frc.chargers.commands.drivetrainCommands.drive

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Velocity
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.commands.commandbuilder.CommandBuilderMarker
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import kotlin.internal.LowPriorityInOverloadResolution
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider

context(CommandBuilder, HeadingProvider)
@JvmName("driveDistanceWithVelocity")
@LowPriorityInOverloadResolution
@CommandBuilderMarker
public fun EncoderDifferentialDrivetrain.driveStraight(
    distance: Distance,
    velocity: Velocity,
    steeringPIDConstants: PIDConstants,
    maxSteeringVelocity: AngularVelocity = DEFAULT_MAX_STEERING_VELOCITY
): Command = runSequentially {
    val initialPosition by getOnceDuringRun { distanceTraveled }
    val initialHeading by getOnceDuringRun { this@HeadingProvider.heading }
    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { this@HeadingProvider.heading },
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
        velocityDrive(velocity, Quantity(0.0), pid.calculateOutput())
    }
}