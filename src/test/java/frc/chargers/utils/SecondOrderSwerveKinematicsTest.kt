package frc.chargers.utils

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.chargerlibexternal.utils.HeadingCorrector
import frc.chargerlibexternal.utils.SecondOrderSwerveKinematics
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.swerve.SuperSwerveDriveKinematics
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class SecondOrderSwerveKinematicsTest {

    @BeforeEach
    fun setup(){
        assert(HAL.initialize(500,0))
    }
    @Test
    fun `to swerve module state`() {
        val secondKinematics = SecondOrderSwerveKinematics(
            Translation2d(0.5, 0.5),
            Translation2d(-0.5, 0.5),
            Translation2d(0.5, -0.5),
            Translation2d(-0.5, -0.5)
        )
        val firstKinematics = SwerveDriveKinematics(
            Translation2d(0.5,0.5),
            Translation2d(-0.5,0.5),
            Translation2d(0.5,-0.5),
            Translation2d(-0.5,-0.5)
        )
        val firstModuleStates = firstKinematics.toSwerveModuleStates(
            ChassisSpeeds(0.0,0.0,1.0)
        )
        val secondModuleStates = secondKinematics.toSwerveModuleState(
            ChassisSpeeds(0.0,0.0,1.0),
            Rotation2d(),
            false
        )

        for (i in 0..<4){
            Assertions.assertEquals(firstModuleStates[i], secondModuleStates.moduleStates[i])
        }


    }

    @Test
    fun toSwerveModuleStateGroup(){

        val speeds = ChassisSpeeds(1.5,0.0,1.0)
        val angle = 0.0.degrees
        val combinedKinematics = SuperSwerveDriveKinematics(
            UnitTranslation2d(0.5.meters,0.5.meters),
            UnitTranslation2d(-0.5.meters,0.5.meters),
            UnitTranslation2d(0.5.meters,-0.5.meters),
            UnitTranslation2d(-0.5.meters,-0.5.meters)
        )
        val secondKinematics = SecondOrderSwerveKinematics(
            Translation2d(0.5, 0.5),
            Translation2d(-0.5, 0.5),
            Translation2d(0.5, -0.5),
            Translation2d(-0.5, -0.5)
        )

        val headingCorrector = HeadingCorrector()

        val combinedModuleStates = combinedKinematics.toSecondOrderModuleStateGroup(
            speeds, angle
        ).toArray()

        val baseModuleStates = secondKinematics.toSwerveModuleState(
            headingCorrector.correctHeading(speeds,angle.asRotation2d()),
            angle.asRotation2d(),
            false
        )

        for (i in 0..<4){
            Assertions.assertEquals(combinedModuleStates[i], baseModuleStates.moduleStates[i])
        }

    }

}