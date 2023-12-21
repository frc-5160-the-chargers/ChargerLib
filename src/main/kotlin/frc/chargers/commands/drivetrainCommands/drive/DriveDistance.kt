package frc.chargers.commands.drivetrainCommands.drive

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Scalar
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.commands.commandbuilder.CommandBuilderMarker
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import kotlin.internal.LowPriorityInOverloadResolution


/**
 * Adds a command to the command builder driving an [EncoderDifferentialDrivetrain] for
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation,
 * while using the best available heading provider given to the drivetrain.
 */
context(CommandBuilder)
@JvmName("driveDistance")
// marks this function with a lower priority than the driveStraight function
// that requires a HeadingProvider context.
@LowPriorityInOverloadResolution
@CommandBuilderMarker
public fun EncoderDifferentialDrivetrain.driveStraightAction(
    distance: Distance,
    power: Double,
    direction: Angle? = null,
    maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER
): Command =
    with(this.gyro ?: this) {
        driveStraightAction(distance, power, direction, maxSteeringPower)
    }

/**
 * Adds a command to the command builder driving an [EncoderDifferentialDrivetrain] for
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder, HeadingProvider)
@JvmName("driveDistance")
public fun EncoderDifferentialDrivetrain.driveStraightAction(
    distance: Distance,
    power: Double,
    direction: Angle? = null,
    maxSteeringPower: Double = DEFAULT_MAX_STEERING_POWER
): Command = runSequentially {
    val initialPosition by getOnceDuringRun { this@EncoderDifferentialDrivetrain.distanceTraveled }
    val initialHeading by getOnceDuringRun { direction ?: this@HeadingProvider.heading }
    val pid by getOnceDuringRun {
        UnitSuperPIDController(
            pidConstants = this@EncoderDifferentialDrivetrain.controlScheme.robotRotationPID,
            getInput = { this@HeadingProvider.heading },
            target = initialHeading,
            outputRange = -Scalar(maxSteeringPower)..Scalar(maxSteeringPower)
        )
    }

    loopUntil(
        if (power > 0) {
            { (distanceTraveled - initialPosition) >= distance }
        } else {
            { (distanceTraveled - initialPosition) <= distance }
        },
        this@EncoderDifferentialDrivetrain
    ) {
        arcadeDrive(power, pid.calculateOutput().siValue)
    }

    runOnce(this@EncoderDifferentialDrivetrain){
        stop()
    }
}


/**
 * Adds a command to the command builder driving an [EncoderHolonomicDrivetrain] for
 * a specified [distance] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder)
@JvmName("driveDistance")
public fun EncoderHolonomicDrivetrain.driveStraightAction(
    distance: Distance,
    power: Double,
    direction: Angle = Angle(0.0)
): Command = runSequentially{
    val initialPosition by getOnceDuringRun{ this@EncoderHolonomicDrivetrain.distanceTraveled }

    loopUntil(
        if (power > 0) {
            { (distanceTraveled - initialPosition) >= distance }
        } else {
            { (distanceTraveled - initialPosition) <= distance }
        },
        this@EncoderHolonomicDrivetrain
    ) {
        topLeft.setDirectionalPower(power,direction)
        topRight.setDirectionalPower(power,direction)
        bottomLeft.setDirectionalPower(power,direction)
        bottomRight.setDirectionalPower(power,direction)
    }

    runOnce(this@EncoderHolonomicDrivetrain){
        stop()
    }
}



