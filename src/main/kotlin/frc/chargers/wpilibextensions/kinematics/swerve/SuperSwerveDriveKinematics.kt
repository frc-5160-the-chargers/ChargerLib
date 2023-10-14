package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.chargers.utils.HeadingCorrector
import frc.chargers.utils.SecondOrderSwerveKinematics
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asAngle
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.*
import kotlin.math.*


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
    private var targetHeading: Angle = Angle(0.0)

    private var previousT: Time = Time(0.0)
    private var offT: Time = Time(0.0)

    private val moduleLocations: Array<Translation2d> = a[
        topLeftLocation.inUnit(meters),
        topRightLocation.inUnit(meters),
        bottomLeftLocation.inUnit(meters),
        bottomRightLocation.inUnit(meters)
    ]

    private val secondKinematics = SecondOrderSwerveKinematics(
        topLeftLocation.inUnit(meters),
        topRightLocation.inUnit(meters),
        bottomLeftLocation.inUnit(meters),
        bottomRightLocation.inUnit(meters)
    )

    private val headingCorrector = HeadingCorrector()





    /**
     * Credits: 5727/4481 second kinematics
     * @see convertSecondOrderChassisSpeeds
     */
    public fun toSecondOrderModuleStateGroup(speeds: ChassisSpeeds, heading: Angle): ModuleStateGroup{
        val standardizedHeading = if(heading < 0.degrees) heading + 360.degrees else heading

        val headingCorrectedSpeeds = headingCorrector.correctHeading(speeds,standardizedHeading.asRotation2d())

        val states = secondKinematics.toSwerveModuleState(headingCorrectedSpeeds,standardizedHeading.asRotation2d())

        return ModuleStateGroup(
            topLeftState = states[0],
            topRightState = states[1],
            bottomLeftState = states[2],
            bottomRightState = states[3]
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







}