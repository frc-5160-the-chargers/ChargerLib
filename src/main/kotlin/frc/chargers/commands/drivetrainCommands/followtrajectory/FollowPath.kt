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
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.constants.drivetrain.DiffDriveControlData
import frc.chargers.pathplannerextensions.asPathPlannerConstants
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
        if (controlScheme.pathAlgorithm == DiffDriveControlData.PathAlgorithm.LTV){
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
                controlData.robotTranslationPID.asPathPlannerConstants(),
                controlData.robotRotationPID.asPathPlannerConstants(),
                hardwareData.maxModuleSpeed.inUnit(meters/seconds),
                sqrt(hardwareData.trackWidth.inUnit(meters).pow(2) + hardwareData.wheelBase.inUnit(meters).pow(2)),
                controlData.pathReplanConfig
            ),
            this@EncoderHolonomicDrivetrain
        ),
        path
    ) { poseEstimator.robotPose.inUnit(meters) }.also(::addCommand)



