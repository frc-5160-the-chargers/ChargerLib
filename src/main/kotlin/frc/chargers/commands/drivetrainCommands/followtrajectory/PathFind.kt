package frc.chargers.commands.drivetrainCommands.followtrajectory

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.commands.PathfindHolonomic
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.pathplannerextensions.asPathPlannerConstants
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Adds a command to the [CommandBuilder] that makes an [EncoderHolonomicDrivetrain]
 * path find to a [UnitPose2d].
 */
context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.pathFindAction(
    targetPose: UnitPose2d,
    constraints: PathConstraints,
    goalEndVel: Velocity = Velocity(0.0),
    rotationDelayDistance: Distance = Distance(0.0)
): Command = PathfindHolonomic(
    targetPose.inUnit(meters),
    constraints,
    goalEndVel.inUnit(meters/seconds),
    { poseEstimator.robotPose.inUnit(meters) },
    { currentSpeeds },
    { speeds: ChassisSpeeds -> velocityDrive(speeds, fieldRelative = false) },
    HolonomicPathFollowerConfig(
        controlData.robotTranslationPID.asPathPlannerConstants(),
        controlData.robotRotationPID.asPathPlannerConstants(),
        hardwareData.maxModuleSpeed.inUnit(meters/ seconds),
        sqrt(hardwareData.trackWidth.inUnit(meters).pow(2) + hardwareData.wheelBase.inUnit(meters).pow(2)),
        controlData.pathReplanConfig
    ),
    rotationDelayDistance.inUnit(meters),
    this
).also(::addCommand)


/**
 * Adds a command to the [CommandBuilder] that makes an [EncoderHolonomicDrivetrain]
 * path find to a [PathPlannerPath], then follow the path to it's end.
 */
context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.pathFindThenFollowPathAction(
    path: PathPlannerPath,
    pathFindConstraints: PathConstraints,
    rotationDelayDistance: Distance = Distance(0.0)
): Command = PathfindThenFollowPathHolonomic(
    path,
    pathFindConstraints,
    { poseEstimator.robotPose.inUnit(meters) },
    { currentSpeeds },
    { speeds: ChassisSpeeds -> velocityDrive(speeds, fieldRelative = false) },
    HolonomicPathFollowerConfig(
        controlData.robotTranslationPID.asPathPlannerConstants(),
        controlData.robotRotationPID.asPathPlannerConstants(),
        hardwareData.maxModuleSpeed.inUnit(meters/ seconds),
        sqrt(hardwareData.trackWidth.inUnit(meters).pow(2) + hardwareData.wheelBase.inUnit(meters).pow(2)),
        controlData.pathReplanConfig
    ),
    rotationDelayDistance.inUnit(meters),
    this
).also(::addCommand)