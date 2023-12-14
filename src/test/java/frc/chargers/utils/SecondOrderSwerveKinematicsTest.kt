package frc.chargers.utils

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.chargerlibexternal.frc4481.HeadingCorrector
import frc.chargerlibexternal.frc4481.SecondOrderSwerveKinematics
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.rotation.asRotation2d
import frc.chargers.wpilibextensions.kinematics.swerve.SuperSwerveDriveKinematics
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import kotlin.math.abs

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
            assertEquals(firstModuleStates[i], secondModuleStates.moduleStates[i])
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
            speeds, angle, fieldRelative = true
        )

        val baseModuleStates = secondKinematics.toSwerveModuleState(
            headingCorrector.correctHeading(speeds,angle.asRotation2d()),
            angle.asRotation2d(),
            /*fieldRelative = */ true
        )

        for (i in 0..<4){
            assertEquals(combinedModuleStates.toArray()[i], baseModuleStates.moduleStates[i])
        }

        assertEquals(combinedModuleStates.topLeftTurnSpeed.siValue, baseModuleStates.turnSpeeds[0])
        assertEquals(combinedModuleStates.topRightTurnSpeed.siValue, baseModuleStates.turnSpeeds[1])
        assertEquals(combinedModuleStates.bottomLeftTurnSpeed.siValue, baseModuleStates.turnSpeeds[2])
        assertEquals(combinedModuleStates.bottomRightTurnSpeed.siValue, baseModuleStates.turnSpeeds[3])

    }

    private infix fun Double.epsilonEquals(other: Double) = abs(this - other) < 1E-9

    @Test
    fun `module states to chassis speeds then back to module states`(){
        val kinematics = SwerveDriveKinematics(
            Translation2d(0.5,0.5),
            Translation2d(-0.5,0.5),
            Translation2d(0.5,-0.5),
            Translation2d(-0.5,-0.5)
        )

        val initialSpeeds = ChassisSpeeds(0.5,0.0,0.5)
        val states = kinematics.toSwerveModuleStates(initialSpeeds)
        val speeds = kinematics.toChassisSpeeds(*states)

        assertEquals(initialSpeeds.vxMetersPerSecond epsilonEquals speeds.vxMetersPerSecond, true)
        assertEquals(initialSpeeds.vyMetersPerSecond epsilonEquals  speeds.vyMetersPerSecond, true)
        assertEquals(initialSpeeds.omegaRadiansPerSecond epsilonEquals speeds.omegaRadiansPerSecond, true)
    }

    @Test
    fun `second kinematics turn speed extraction`(){
        val kinematics = SuperSwerveDriveKinematics(
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

        val speeds = ChassisSpeeds(1.5,0.0,0.5)
        val secondOrderOutput = kinematics.toSecondOrderModuleStateGroup(
            speeds,
            0.0.degrees,
            fieldRelative = false
        )

        val firstOrderArray = kinematics.toSwerveModuleStates(
            speeds
        )

        val testTurnSpeeds = secondKinematics.extractTurnSpeeds(
            firstOrderArray,
            speeds.omegaRadiansPerSecond
        ).toList()

        val initialTurnSpeeds = listOf(
            secondOrderOutput.topLeftTurnSpeed.siValue,
            secondOrderOutput.topRightTurnSpeed.siValue,
            secondOrderOutput.bottomLeftTurnSpeed.siValue,
            secondOrderOutput.bottomRightTurnSpeed.siValue
        )

        for (i in 0..<4){
            assertEquals(testTurnSpeeds[i] epsilonEquals initialTurnSpeeds[i], true)
        }

    }



    /*
    @Test
    fun `FRC 95 kinematics should be equivalent to 4481 second kinematics`(){
        val frc95kinematics = BetterSwerveKinematics(
            Translation2d(0.5,0.5),
            Translation2d(-0.5,0.5),
            Translation2d(0.5,-0.5),
            Translation2d(-0.5,-0.5)
        )

        val frc4481kinematics = SecondOrderSwerveKinematics(
            Translation2d(0.5, 0.5),
            Translation2d(-0.5, 0.5),
            Translation2d(0.5, -0.5),
            Translation2d(-0.5, -0.5)
        )

        val frc95StateOutput: Array<BetterSwerveModuleState> = frc95kinematics.toSwerveModuleStates(
            ChassisSpeeds(0.5,0.0,0.5)
        )

        val frc4481Output: SecondOrderSwerveKinematics.Output = frc4481kinematics.toSwerveModuleState(
            ChassisSpeeds(0.5,0.0,0.5),
            Rotation2d(0.0),
            false
        )

        for (i in 0..<4){
            assertEquals(frc95StateOutput[i].speedMetersPerSecond - frc4481Output.moduleStates[i].speedMetersPerSecond < 1E-9, true)
            assertEquals(frc95StateOutput[i].angle.radians - frc4481Output.moduleStates[i].angle.radians < 1E-9, true)
            assertEquals(frc95StateOutput[i].omegaRadPerSecond - frc4481Output.turnSpeeds[i] < 1E-9, true)
        }



    }

     */

}