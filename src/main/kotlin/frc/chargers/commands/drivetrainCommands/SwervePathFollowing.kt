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
import frc.chargers.utils.PathPlannerAutoContext
import frc.chargers.utils.asPathPlannerConstants
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.ofUnit

/*
 * Stores all the commands related to path following with the [EncoderHolonomicDrivetrain],
 * using pathplannerlib.
 */



/**
 * Makes an [EncoderHolonomicDrivetrain] follow a designated path, using [PPSwerveControllerCommand] and a [PathPlannerTrajectory].
 */
context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.followPath(
    trajectory: PathPlannerTrajectory,
    isFirstPath: Boolean = false,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants
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
            constants = translationOffsetConstants
        },
        PIDController(0.0,0.0,0.0).apply{
            constants = translationOffsetConstants
        },
        PIDController(0.0,0.0,0.0).apply{
            constants = rotationOffsetConstants
        },
        ::swerveDrive,
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
public fun EncoderHolonomicDrivetrain.followPath(
    trajectoryName: String,
    pathConstraints: LinearTrapezoidProfile.Constraints,
    isFirstPath: Boolean = false,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants,
): Command = followPath(
    PathPlanner.loadPath(
        trajectoryName,
        PathConstraints(
            pathConstraints.maxVelocity.siValue,
            pathConstraints.maxAcceleration.siValue
        )
    ),
    isFirstPath,
    translationOffsetConstants,
    rotationOffsetConstants
)


context(CommandBuilder)
public inline fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    trajectories: List<PathPlannerTrajectory>,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants,
    eventsBlock: PathPlannerAutoContext.() -> Unit
): Command{
    // NOTE: this@runPathPlannerAuto denotes the "this" of the followPath function
    // instead of the CommandBuilder's this,
    // which refers to the DRIVETRAIN.
    val autoBuilder = SwerveAutoBuilder(
        {robotPose.inUnit(meters)},  // Pose2d supplier
        {resetPose(it.ofUnit(meters))},  // Pose2d consumer, used to reset odometry at the beginning of auto
        translationOffsetConstants.asPathPlannerConstants(),  // PID constants to correct for translation error (used to create the X and Y PID controllers)
        rotationOffsetConstants.asPathPlannerConstants(),  // PID constants to correct for rotation error (used to create the rotation controller)
        ::swerveDrive,  // chassis speeds consumer
        PathPlannerAutoContext().apply(eventsBlock).javaEventMap,
        true,  // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true,
        this@runPathPlannerAuto // The drive subsystem. Used to properly set the requirements of path following commands
    )

    return autoBuilder.fullAuto(trajectories).also(commands::add)
}

context(CommandBuilder)
public inline fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    pathGroupName: String,
    pathConstraints: LinearTrapezoidProfile.Constraints,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants,
    eventsBlock: PathPlannerAutoContext.() -> Unit
): Command = runPathPlannerAuto(
    PathPlanner.loadPathGroup(
        pathGroupName,
        PathConstraints(
            pathConstraints.maxVelocity.siValue,
            pathConstraints.maxAcceleration.siValue
        )
    ),
    translationOffsetConstants,
    rotationOffsetConstants,
    eventsBlock
)

context(CommandBuilder)
public inline fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    pathGroupName: String,
    allPathConstraints: List<LinearTrapezoidProfile.Constraints>,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants,
    eventsBlock: PathPlannerAutoContext.() -> Unit
): Command = runPathPlannerAuto(
    PathPlanner.loadPathGroup(
        pathGroupName,
        allPathConstraints.map {
            PathConstraints(
                it.maxVelocity.siValue,
                it.maxAcceleration.siValue
            )
        }
    ),
    translationOffsetConstants,
    rotationOffsetConstants,
    eventsBlock
)










