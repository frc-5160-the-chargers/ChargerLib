package frc.chargers.commands.drivetrainCommands.followtrajectory

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.commands.FollowPathLTV
import com.pathplanner.lib.commands.FollowPathRamsete
import com.pathplanner.lib.commands.FollowPathWithEvents
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystemutils.differentialdrive.DiffDriveControl
import frc.chargers.utils.*
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * Adds a command to the [CommandBuilder] which makes an [EncoderDifferentialDrivetrain]
 * follow a designated path, utilizing a [PathPlannerPath].
 *
 * The algorithm(LTV vs. Ramsete) depends on the configuration
 * of the [EncoderDifferentialDrivetrain]'s controlScheme.
 */
context(CommandBuilder)
public fun EncoderDifferentialDrivetrain.followPathAction(path: PathPlannerPath): Command =
    FollowPathWithEvents(
        if (controlScheme.pathAlgorithm == DiffDriveControl.PathAlgorithm.LTV){
            FollowPathLTV(
                path,
                { poseEstimator.robotPose.inUnit(meters) },
                { currentSpeeds },
                { speeds -> velocityDrive(speeds) },
                ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                controlScheme.pathReplanConfig,
                this@EncoderDifferentialDrivetrain
            )
        }else{
            FollowPathRamsete(
                path,
                { poseEstimator.robotPose.inUnit(meters) },
                { currentSpeeds },
                { speeds -> velocityDrive(speeds) },
                controlScheme.pathReplanConfig,
                this@EncoderDifferentialDrivetrain
            )
        },
        path
    ) { poseEstimator.robotPose.inUnit(meters) }.also(::addCommand)


/**
 * Adds a command to the [CommandBuilder] which makes an [EncoderHolonomicDrivetrain]
 * follow a designated path, utilizing a [PathPlannerPath].
 */
context(CommandBuilder)
public fun EncoderHolonomicDrivetrain.followPathAction(path: PathPlannerPath): Command =
    FollowPathWithEvents(
        FollowPathHolonomic(
            path,
            {poseEstimator.robotPose.inUnit(meters)},
            {currentSpeeds},
            { speeds -> velocityDrive(speeds, fieldRelative = false) },
            HolonomicPathFollowerConfig(
                controlScheme.robotTranslationPID.asPathPlannerConstants(),
                controlScheme.robotRotationPID.asPathPlannerConstants(),
                constants.maxModuleSpeed.inUnit(meters/seconds),
                sqrt(constants.trackWidth.inUnit(meters).pow(2) + constants.wheelBase.inUnit(meters).pow(2)),
                controlScheme.pathReplanConfig
            ),
            this@EncoderHolonomicDrivetrain
        ),
        path
    ) { poseEstimator.robotPose.inUnit(meters) }.also(::addCommand)



