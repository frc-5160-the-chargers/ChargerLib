package frc.chargers.commands.drivetrainCommands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.RamseteAutoBuilder
import com.pathplanner.lib.commands.PPRamseteCommand
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.CommandBuilder
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.utils.PathPlannerAutoContext
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.ofUnit
import kotlin.internal.LowPriorityInOverloadResolution

context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.followPath(
    trajectory: PathPlannerTrajectory,
    isFirstPath: Boolean = false,
    ramseteController: RamseteController = RamseteController()
): Command = runSequentially{
    // NOTE: this@followPath denotes the "this" of the followPath function
    // instead of the CommandBuilder's this,
    // which refers to the DRIVETRAIN.
    runOnce(this@followPath){
        if(isFirstPath){
            this@followPath.resetPose(trajectory.initialPose.ofUnit(meters))
        }
    }

    +PPRamseteCommand(
        trajectory,
        {robotPose.inUnit(meters)},
        ramseteController,
        kinematics,
        {leftSpeed: Double,rightSpeed: Double ->
            velocityDrive(leftSpeed.ofUnit(meters/ seconds),rightSpeed.ofUnit(meters/ seconds))},
        true,
        this@followPath
    )
}

context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.followPath(
    trajectoryName: String,
    pathConstraints: LinearTrapezoidProfile.Constraints,
    isFirstPath: Boolean = false,
    ramseteController: RamseteController = RamseteController()
): Command = followPath(
    PathPlanner.loadPath(
        trajectoryName,
        PathConstraints(
            pathConstraints.maxVelocity.siValue,
            pathConstraints.maxAcceleration.siValue
        )
    ),
    isFirstPath,
    ramseteController
)

context(CommandBuilder)
public inline fun EncoderDifferentialDrivetrain.runPathPlannerAuto(
    trajectories: List<PathPlannerTrajectory>,
    ramseteController: RamseteController = RamseteController(),
    eventsBlock: PathPlannerAutoContext.() -> Unit
): Command{
    val autoBuilder = RamseteAutoBuilder(
        {robotPose.inUnit(meters)},
        {resetPose(it.ofUnit(meters))},
        ramseteController,
        kinematics,
        {
            leftSpeed: Double, rightSpeed: Double -> velocityDrive(
            leftSpeed.ofUnit(meters/seconds),
            rightSpeed.ofUnit(meters/seconds))
        },
        PathPlannerAutoContext().apply(eventsBlock).eventMap,
        this@runPathPlannerAuto
    )

    return autoBuilder.fullAuto(trajectories).also(commands::add)
}

context(CommandBuilder)
public inline fun EncoderDifferentialDrivetrain.runPathPlannerAuto(
    pathGroupName: String,
    pathConstraints: LinearTrapezoidProfile.Constraints,
    ramseteController: RamseteController = RamseteController(),
    eventsBlock: PathPlannerAutoContext.() -> Unit
): Command = runPathPlannerAuto(
    PathPlanner.loadPathGroup(
        pathGroupName,
        PathConstraints(
            pathConstraints.maxVelocity.siValue,
            pathConstraints.maxAcceleration.siValue
        )
    ),
    ramseteController,
    eventsBlock
)

context(CommandBuilder)
public inline fun EncoderDifferentialDrivetrain.runPathPlannerAuto(
    pathGroupName: String,
    allPathConstraints: List<LinearTrapezoidProfile.Constraints>,
    ramseteController: RamseteController = RamseteController(),
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
    ramseteController,
    eventsBlock
)





