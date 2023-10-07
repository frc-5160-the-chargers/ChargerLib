package frc.chargers.commands.drivetrainCommands

import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.SwerveAutoBuilder
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.CommandBuilder
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.constants
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.utils.*
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.kinematics.rotationSpeed
import frc.chargers.wpilibextensions.kinematics.xVelocity
import frc.chargers.wpilibextensions.kinematics.yVelocity
import kotlin.internal.LowPriorityInOverloadResolution

/*
 * Stores all the commands related to path following with the [EncoderHolonomicDrivetrain],
 * using pathplannerlib.
 */



context(CommandBuilder, PathData)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.followPath(
    trajectoryName: String,
    isFirstPath: Boolean = false
): Command = followPath(
    PathPlanner.loadPath(trajectoryName, constraints),
    translationConstants,rotationConstants,isFirstPath
)
/**
 * Makes an [EncoderHolonomicDrivetrain] follow a designated path, using [PPSwerveControllerCommand] and a [PathPlannerTrajectory].
 */
context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.followPath(
    trajectory: PathPlannerTrajectory,
    translationConstants: PIDConstants,
    rotationConstants: PIDConstants,
    isFirstPath: Boolean = false,
): Command = runSequentially{
    // NOTE: this@followPath denotes the "this" of the followPath function
    // instead of the CommandBuilder's this,
    // which refers to the DRIVETRAIN.
    runOnce(this@followPath){
        if(isFirstPath){
            this@followPath.resetPose(trajectory.initialHolonomicPose.ofUnit(meters))
        }
    }

    +PPSwerveControllerCommand(
        trajectory,
        {robotPose.inUnit(meters)},
        PIDController(0.0,0.0,0.0).apply{
            constants = translationConstants
        },
        PIDController(0.0,0.0,0.0).apply{
            constants = translationConstants
        },
        PIDController(0.0,0.0,0.0).apply{
            constants = rotationConstants
        },
        {speeds ->
        velocityDrive(speeds.xVelocity,-speeds.yVelocity,speeds.rotationSpeed)
        },
        true,
        this@followPath
    )
}


/**
 * Makes an [EncoderHolonomicDrivetrain] follow a designated path, using [PPSwerveControllerCommand].
 *
 * IMPORTANT: use PathPlannerServer.
 *
 * Utilizes a [trajectoryName] and [LinearTrapezoidProfile.Constraints] instead of a [PathPlannerTrajectory].
 */
context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.followPath(
    trajectoryName: String,
    translationConstants: PIDConstants,
    rotationConstants: PIDConstants,
    pathConstraints: PathConstraints,
    isFirstPath: Boolean = false,
): Command = followPath(
    PathPlanner.loadPath(
        trajectoryName,
        pathConstraints
    ),
    translationConstants,
    rotationConstants,
    isFirstPath
)


context(CommandBuilder,PathData)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    pathGroupName: String,
    eventsBlock: MappableContext<String, Command>.() -> Unit
): Command = runPathPlannerAuto(
    pathGroupName,
    translationConstants,
    rotationConstants,
    allPathConstraints = a[constraints] + otherConstraints.toTypedArray(),
    eventsBlock
)
context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    trajectories: List<PathPlannerTrajectory>,
    translationConstants: PIDConstants,
    rotationConstants: PIDConstants,
    eventsBlock: MappableContext<String, Command>.() -> Unit
): Command{
    // NOTE: this@runPathPlannerAuto denotes the "this" of the followPath function
    // instead of the CommandBuilder's this,
    // which refers to the DRIVETRAIN.
    val autoBuilder = SwerveAutoBuilder(
        {robotPose.inUnit(meters)},  // Pose2d supplier
        {resetPose(it.ofUnit(meters))},  // Pose2d consumer, used to reset odometry at the beginning of auto
        translationConstants.asPathPlannerConstants(),  // PID constants to correct for translation error (used to create the X and Y PID controllers)
        rotationConstants.asPathPlannerConstants(),  // PID constants to correct for rotation error (used to create the rotation controller)
        ::velocityDrive,  // chassis speeds consumer
        MappableContext<String,Command>().apply(eventsBlock).map,
        true,  // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true,
        this@runPathPlannerAuto // The drive subsystem. Used to properly set the requirements of path following commands
    )

    return autoBuilder.fullAuto(trajectories).also(commands::add)
}

context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    pathGroupName: String,
    translationConstants: PIDConstants,
    rotationConstants: PIDConstants,
    vararg allPathConstraints: PathConstraints,
    eventsBlock: MappableContext<String,Command>.() -> Unit
): Command = runPathPlannerAuto(
    PathPlanner.loadPathGroup(
        pathGroupName,
        listOf(*allPathConstraints)
    ),
    translationConstants,
    rotationConstants,
    eventsBlock
)











