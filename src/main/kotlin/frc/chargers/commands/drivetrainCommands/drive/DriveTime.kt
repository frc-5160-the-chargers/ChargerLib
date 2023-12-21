package frc.chargers.commands.drivetrainCommands.drive

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.commands.commandbuilder.CommandBuilderMarker
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.DifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
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
        UnitSuperPIDController(
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
): Command = with (this as HeadingProvider){
    driveStraightAction(time, power, steeringPIDConstants,direction, maxSteeringPower)
}

/**
 * Adds a command to the command builder driving a swerve-drive robot for
 * a specified [time] with a specified [power], using
 * PID to ensure a straight drive without deviation.
 */
context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.driveStraightAction(
    time: Time,
    power: Double,
    direction: Angle
): Command = runSequentially{

    loopFor(time,this@EncoderHolonomicDrivetrain){
        topLeft.setDirectionalPower(power,direction)
        topRight.setDirectionalPower(power,direction)
        bottomLeft.setDirectionalPower(power,direction)
        bottomRight.setDirectionalPower(power,direction)
    }

    runOnce(this@EncoderHolonomicDrivetrain){
        stop()
    }

}









/*


context(CommandBuilder, TurnPIDConstants)
@JvmName("driveDistanceWithVelocity")
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.driveStraight(distance: Distance, velocity: Velocity, maxSteeringVelocity: AngularVelocity = DEFAULT_MAX_STEERING_VELOCITY): Command =
    driveStraight(distance, velocity, turnPIDConstants, maxSteeringVelocity)

context(CommandBuilder)
@JvmName("driveDistanceWithVelocity")
@LowPriorityInOverloadResolution
@CommandBuilderMarker
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


context(CommandBuilder)
@JvmName("swerveDriveTime")
@LowPriorityInOverloadResolution
@CommandBuilderMarker
public fun EncoderHolonomicDrivetrain.driveStraight(
    direction: Angle = gyro?.heading ?: this.heading,
    time: Time,
    power: Double
): Command =
    loopFor(time,this){
        topLeft.setDirectionalPower(power,direction)
        topRight.setDirectionalPower(power,direction)
        bottomLeft.setDirectionalPower(power,direction)
        bottomRight.setDirectionalPower(power,direction)
    }

context(CommandBuilder)
@JvmName("swerveDriveDistance")
@LowPriorityInOverloadResolution
@CommandBuilderMarker
public fun EncoderHolonomicDrivetrain.driveStraight(
    direction: Angle = gyro?.heading ?: this.heading,
    distance: Distance,
    power: Double
): Command =
    loopWhile({distanceTraveled < distance},this){
        topLeft.setDirectionalPower(power,direction)
        topRight.setDirectionalPower(power,direction)
        bottomLeft.setDirectionalPower(power,direction)
        bottomRight.setDirectionalPower(power,direction)
    }


private fun test(drive: EncoderHolonomicDrivetrain) = buildCommand {
    runOnce{
        with(HeadingProvider{Angle(0.0)}){
            drive.driveStraight(
                time = Time(0.0),
                2.0,
                steeringPIDConstants = PIDConstants(0.0,0.0,0.0)
            )
        }
    }
}

 */
