package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
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



    /**
     * Credits: 5727/4481 second kinematics
     * @see convertSecondOrderChassisSpeeds
     */
    public fun toSecondOrderModuleStateGroup(speeds: ChassisSpeeds, heading: Angle, correctHeading: Boolean = false): ModuleStateGroup{
        val arr = convertSecondOrderChassisSpeeds(
            correctHeading(speeds,heading),
            heading.asRotation2d()
        )

        return ModuleStateGroup(
            topLeftState = arr[0],
            topRightState = arr[1],
            bottomLeftState = arr[2],
            bottomRightState = arr[3]
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
     * Credits: [4481 second order swerve drive kinematics](https://github.com/FRC-4481-Team-Rembrandts/4481-Stock-Robot-2023-Public/blob/1988b5b9fb01f0fb2fd15d67197a3968efbd52d5/src/main/java/frc/team4481/lib/swerve/SecondOrderSwerveKinematics.java)
     */
    private fun convertSecondOrderChassisSpeeds(desiredSpeed: ChassisSpeeds, robotHeading: Rotation2d): Array<SwerveModuleState> {
        val firstOrderInputMatrix = Matrix(Nat.N3(), Nat.N1())
        val firstOrderMatrix = Matrix(Nat.N2(), Nat.N3())
        val secondOrderInputMatrix = Matrix(Nat.N4(), Nat.N1())
        val secondOrderMatrix = Matrix(Nat.N2(), Nat.N4())
        val rotationMatrix = Matrix(Nat.N2(), Nat.N2())

        firstOrderInputMatrix.set(0, 0, desiredSpeed.vxMetersPerSecond)
        firstOrderInputMatrix.set(1, 0, desiredSpeed.vyMetersPerSecond)
        firstOrderInputMatrix.set(2, 0, desiredSpeed.omegaRadiansPerSecond)

        secondOrderInputMatrix[2, 0] = desiredSpeed.omegaRadiansPerSecond.pow(2.0)

        firstOrderMatrix[0, 0] = 1.0
        firstOrderMatrix[1, 1] = 1.0

        secondOrderMatrix[0, 0] = 1.0
        secondOrderMatrix[1, 1] = 1.0

        val swerveModuleStates = Array(4){SwerveModuleState(0.0,Rotation2d())}
        val moduleTurnSpeeds = DoubleArray(4)

        for (i in 0..3) {
            val moduleAngle = Rotation2d(
                kotlin.math.atan2(
                    moduleLocations[i].y,
                    moduleLocations[i].x
                )
            ) //Angle that the module location vector makes with respect to the robot
            val moduleAngleFieldCentric =
                moduleAngle + robotHeading //Angle that the module location vector makes with respect to the field
            val moduleX = moduleLocations[i].norm * cos(moduleAngleFieldCentric.radians)
            val moduleY = moduleLocations[i].norm * sin(moduleAngleFieldCentric.radians)
            firstOrderMatrix[0, 2] = -moduleY //-r_y
            firstOrderMatrix[1, 2] = moduleX //r_x
            val firstOrderOutput = firstOrderMatrix * firstOrderInputMatrix
            val moduleHeading = kotlin.math.atan2(firstOrderOutput[1, 0], firstOrderOutput[0, 0])
            val moduleSpeed = sqrt(firstOrderOutput.elementPower(2).elementSum())
            secondOrderMatrix[0, 2] = -moduleX
            secondOrderMatrix[0, 3] = -moduleY
            secondOrderMatrix[1, 2] = -moduleY
            secondOrderMatrix[1, 3] = moduleX
            rotationMatrix[0, 0] = cos(moduleHeading)
            rotationMatrix[0, 1] = sin(moduleHeading)
            rotationMatrix[1, 0] = -sin(moduleHeading)
            rotationMatrix[1, 1] = cos(moduleHeading)
            val secondOrderOutput = rotationMatrix.times(secondOrderMatrix.times(secondOrderInputMatrix))
            swerveModuleStates[i] = SwerveModuleState(moduleSpeed, Rotation2d(moduleHeading).minus(robotHeading))
            moduleTurnSpeeds[i] = secondOrderOutput[1, 0] / moduleSpeed - desiredSpeed.omegaRadiansPerSecond
        }

        return swerveModuleStates
    }

    /**
     * Credits: 4481, 5727 codebase
     */
    private fun correctHeading(desiredSpeed: ChassisSpeeds, inputHeading: Angle): ChassisSpeeds {
        //Determine time interval
        val currentT: Time = fpgaTimestamp()
        val dt: Time = currentT - previousT
        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        val vr = desiredSpeed.rotationSpeed
        if (vr > 0.01.ofUnit(radians/seconds) || vr < -(0.01.ofUnit(radians/seconds)) ) {
            offT = currentT
            targetHeading = inputHeading
            return desiredSpeed
        }
        if (currentT - offT < 0.5.seconds) {
            targetHeading = inputHeading
            return desiredSpeed
        }
        //Determine target and current heading
        targetHeading += vr * dt
        //Calculate the change in heading that is needed to achieve the target
        val deltaHeading = targetHeading - inputHeading
        if (abs(deltaHeading) < 0.05.degrees) {
            return desiredSpeed
        }
        val correctedVr = deltaHeading / dt * 0.05
        previousT = currentT
        return ChassisSpeeds(desiredSpeed.xVelocity, desiredSpeed.yVelocity, correctedVr)
    }


}