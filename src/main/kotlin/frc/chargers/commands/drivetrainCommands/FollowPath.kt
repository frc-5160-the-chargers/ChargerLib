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
import frc.chargers.commands.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.constants
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.ofUnit
import com.pathplanner.lib.auto.PIDConstants as PathPlannerPIDConstants

public fun PIDConstants.asPathPlannerConstants(): PathPlannerPIDConstants =
    PathPlannerPIDConstants(kP,kI,kD)


context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.followPath(
    trajectory: PathPlannerTrajectory,
    isFirstPath: Boolean = false,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants
): Command{
    return buildCommand{
        runOnce(this@EncoderHolonomicDrivetrain){
            if(isFirstPath){
                this@EncoderHolonomicDrivetrain.resetPose(trajectory.initialHolonomicPose.ofUnit(meters))
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
            ::velocityDrive,
            true,
            this@EncoderHolonomicDrivetrain
        )
    }
}

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

public class PathPlannerAutoContext{
    public var eventMap: MutableMap<String,Command> = mutableMapOf()

    public val javaEventMap: HashMap<String,Command>
        get() = HashMap(eventMap)
    public fun onEvent(eventName: String, command: Command){
        eventMap[eventName] = command
    }

    public inline fun onEvent(
        eventName: String,
        context: CommandBuilder.() -> Unit
    ): Unit = onEvent(eventName, buildCommand(context))
}

context(CommandBuilder)
public inline fun EncoderHolonomicDrivetrain.runPathPlannerAuto(
    trajectories: List<PathPlannerTrajectory>,
    translationOffsetConstants: PIDConstants,
    rotationOffsetConstants: PIDConstants,
    eventsBlock: PathPlannerAutoContext.() -> Unit
): Command{

    val autoBuilder = SwerveAutoBuilder(
        {robotPose.inUnit(meters)},  // Pose2d supplier
        {resetPose(it.ofUnit(meters))},  // Pose2d consumer, used to reset odometry at the beginning of auto
        translationOffsetConstants.asPathPlannerConstants(),  // PID constants to correct for translation error (used to create the X and Y PID controllers)
        rotationOffsetConstants.asPathPlannerConstants(),  // PID constants to correct for rotation error (used to create the rotation controller)
        ::velocityDrive,  // Module states consumer used to output to the drive subsystem
        PathPlannerAutoContext().apply(eventsBlock).javaEventMap,
        true,  // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true,
        this@EncoderHolonomicDrivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    )

    return autoBuilder.fullAuto(trajectories)
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










