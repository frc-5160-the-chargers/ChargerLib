package frc.chargers.utils

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.swerve.SuperSwerveDriveKinematics
import junit.framework.TestCase.assertEquals
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

internal class SecondOrderSwerveKinematicsTest {
    @Test
    fun toSwerveModuleState() {
        val secondKinematics = SecondOrderSwerveKinematics(
            Translation2d(0.5,0.5),
            Translation2d(-0.5,0.5),
            Translation2d(0.5,-0.5),
            Translation2d(-0.5,-0.5)
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
            Rotation2d()
        )

        for (i in 0..<4){
            Assertions.assertEquals(firstModuleStates[i], secondModuleStates[i])
        }


    }

    @Test
    fun toSwerveModuleStateGroup(){
        /*
        val speeds = ChassisSpeeds(1.5,0.0,1.0)
        val angle = 0.0.degrees
        val combinedKinematics = SuperSwerveDriveKinematics(
            UnitTranslation2d(0.5.meters,0.5.meters),
            UnitTranslation2d(-0.5.meters,0.5.meters),
            UnitTranslation2d(0.5.meters,-0.5.meters),
            UnitTranslation2d(-0.5.meters,-0.5.meters)
        )
        val secondKinematics = SecondOrderSwerveKinematics(
            Translation2d(0.5,0.5),
            Translation2d(-0.5,0.5),
            Translation2d(0.5,-0.5),
            Translation2d(-0.5,-0.5)
        )

        val headingCorrector = HeadingCorrector()

        val combinedModuleStates = combinedKinematics.toSecondOrderModuleStateGroup(
            speeds, 0.0.degrees
        ).toArray()

        val baseModuleStates = secondKinematics.toSwerveModuleState(
            headingCorrector.correctHeading(speeds,angle.asRotation2d()),
            angle.asRotation2d()
        )

        for (i in 0..<4){
            Assertions.assertEquals(combinedModuleStates[i], baseModuleStates[i])
        }

         */



    }
}