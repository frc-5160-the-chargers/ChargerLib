package frc.chargers.commands.drivetrainCommands.drive

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.constants.drivetrain.DEFAULT_MAX_STEERING_VELOCITY
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain


/**
 * Adds a command to the command builder driving an [EncoderDifferentialDrivetrain] for
 * a specified [distance] with a specified [velocity], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder)
@JvmName("driveDistanceWithVelocity")
public fun EncoderDifferentialDrivetrain.driveStraightAction(
    distance: Distance,
    velocity: Velocity,
    direction: Angle? = null,
    steeringPIDConstants: PIDConstants,
    maxSteeringVelocity: AngularVelocity = DEFAULT_MAX_STEERING_VELOCITY
): Command = runSequentially {
    fun getHeading(): Angle{
        return this@EncoderDifferentialDrivetrain.gyro?.heading ?: this@EncoderDifferentialDrivetrain.heading
    }
    val initialPosition by getOnceDuringRun { distanceTraveled }
    val initialHeading by getOnceDuringRun { direction ?: getHeading() }
    val keepHeadingPID by getOnceDuringRun {
        SuperPIDController(
            pidConstants = steeringPIDConstants,
            getInput = { gyro?.heading ?: this@EncoderDifferentialDrivetrain.heading },
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
        this@driveStraightAction
    ) {
        velocityDrive(velocity, Quantity(0.0), keepHeadingPID.calculateOutput())
    }
}


/**
 * Adds a command to the command builder driving an [EncoderHolonomicDrivetrain] for
 * a specified [distance] with a specified [velocity], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder)
@JvmName("driveDistanceWithVelocity")
public fun EncoderHolonomicDrivetrain.driveStraightAction(
    distance: Distance,
    velocity: Velocity,
    direction: Angle = Angle(0.0),
    fieldRelative: Boolean = false
): Command = runSequentially{
    val initialPosition by getOnceDuringRun { distanceTraveled }

    loopUntil(
        if (velocity > Velocity(0.0)) {
            { (distanceTraveled - initialPosition) >= distance }
        } else {
            { (distanceTraveled - initialPosition) <= distance }
        },
        this@driveStraightAction
    ) {
        val angularVelocity = velocity / (hardwareData.wheelDiameter / 2)
        if (fieldRelative){
            val fieldRelativeDelta: Angle = gyro?.heading ?: this@EncoderHolonomicDrivetrain.heading
            topLeft.setDirectionalVelocity(angularVelocity,direction - fieldRelativeDelta )
            topRight.setDirectionalVelocity(angularVelocity,direction - fieldRelativeDelta )
            bottomLeft.setDirectionalVelocity(angularVelocity,direction - fieldRelativeDelta )
            bottomRight.setDirectionalVelocity(angularVelocity,direction - fieldRelativeDelta )
        }else{
            topLeft.setDirectionalVelocity(angularVelocity,direction)
            topRight.setDirectionalVelocity(angularVelocity,direction)
            bottomLeft.setDirectionalVelocity(angularVelocity,direction)
            bottomRight.setDirectionalVelocity(angularVelocity,direction)
        }
    }
}
