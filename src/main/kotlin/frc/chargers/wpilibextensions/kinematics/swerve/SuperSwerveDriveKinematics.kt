package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.chargerlibexternal.utils.HeadingCorrector
import frc.chargerlibexternal.utils.SecondOrderSwerveKinematics
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d


/**
 * A wrapper for WPILib's [SwerveDriveKinematics], with support for second kinematics,
 * and integration with [ModuleStateGroup].
 *
 * Big Thanks to WitherSlayer67 for helping with this!
 *
 * Credits: [5727 codebase](https://github.com/FRC5727/SwervyBoi/blob/THOR2023), [4481 codebase](https://github.com/FRC-4481-Team-Rembrandts/4481-Stock-Robot-2023-Public/tree/1988b5b9fb01f0fb2fd15d67197a3968efbd52d5)
 */
public class SuperSwerveDriveKinematics(
    topLeftLocation: UnitTranslation2d,
    topRightLocation: UnitTranslation2d,
    bottomLeftLocation: UnitTranslation2d,
    bottomRightLocation: UnitTranslation2d
): SwerveDriveKinematics(
    topLeftLocation.inUnit(meters),
    topRightLocation.inUnit(meters),
    bottomLeftLocation.inUnit(meters),
    bottomRightLocation.inUnit(meters)
){

    private val secondKinematics = SecondOrderSwerveKinematics(
        topLeftLocation.inUnit(meters),
        topRightLocation.inUnit(meters),
        bottomLeftLocation.inUnit(meters),
        bottomRightLocation.inUnit(meters)
    )

    private val headingCorrector = HeadingCorrector()






    /**
     * Credits: 5727/4481 second kinematics
     * @see SecondOrderSwerveKinematics
     */
    public fun toSecondOrderModuleStateGroup(speeds: ChassisSpeeds, heading: Angle, fieldRelative: Boolean = true): SecondOrderModuleStateGroup{

        val headingCorrectedSpeeds = headingCorrector.correctHeading(speeds,heading.asRotation2d())

        val output = secondKinematics.toSwerveModuleState(headingCorrectedSpeeds,heading.asRotation2d(), fieldRelative)
        val states = output.moduleStates

        // lolll.... 4481's second order kinematics is returning NaN for turn speeds...
        val turnSpeeds = output.turnSpeeds.map{
            if (it.isNaN()){
                AngularVelocity(0.0)
            }else{
                it.ofUnit(radians/seconds)
            }
        }


        return SecondOrderModuleStateGroup(
            topLeftState = states[0],
            topRightState = states[1],
            bottomLeftState = states[2],
            bottomRightState = states[3],
            turnSpeeds[0], turnSpeeds[1], turnSpeeds[2], turnSpeeds[3]
        )
    }

    public fun toFirstOrderModuleStateGroup(speeds: ChassisSpeeds): ModuleStateGroup{
        val arr = toSwerveModuleStates(speeds)
        return ModuleStateGroup(
            topLeftState = arr[0],
            topRightState = arr[1],
            bottomLeftState = arr[2],
            bottomRightState = arr[3]
        )
    }

    public fun toChassisSpeeds(stateGroup: ModuleStateGroup): ChassisSpeeds =
        toChassisSpeeds(
            stateGroup.topLeftState,
            stateGroup.topRightState,
            stateGroup.bottomLeftState,
            stateGroup.bottomRightState
        )

    /**
     * Extracts the turn speeds out of an existing [ModuleStateGroup], with a target rotation speed.
     */
    public fun extractTurnSpeeds(stateGroup: ModuleStateGroup, rotationSpeed: AngularVelocity): List<AngularVelocity>{
        val output = secondKinematics.extractTurnSpeeds(
            arrayOf(stateGroup.topLeftState, stateGroup.topRightState, stateGroup.bottomLeftState, stateGroup.bottomRightState),
            rotationSpeed.inUnit(radians/seconds)
        )

        return output.map{
            if (it.isNaN()){
                AngularVelocity(0.0)
            }else{
                it.ofUnit(radians/seconds)
            }
        }
    }







}