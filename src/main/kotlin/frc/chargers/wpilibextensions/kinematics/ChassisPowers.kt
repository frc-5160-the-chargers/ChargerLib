// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d


public class ChassisPowers(
    public var xPower: Double = 0.0,
    public var yPower: Double = 0.0,
    public var rotationPower: Double = 0.0
){
    public fun asFieldRelative(robotAngle: Angle): ChassisPowers = FieldRelativeChassisPowers(xPower,yPower,rotationPower,robotAngle)
}

public fun FieldRelativeChassisPowers(
    xPower: Double,
    yPower: Double,
    rotationPower: Double,
    robotAngle: Angle): ChassisPowers =
    ChassisPowers(
        xPower * cos(robotAngle) + yPower * sin(robotAngle),
        -xPower * sin(robotAngle) + yPower * cos(robotAngle),
        rotationPower)


/**
 * A function used to correct for drift on swerve drive when simultaneously rotating and driving in a singular direction.
 *
 * Credits: [254](https://github.com/Team254/FRC-2022-Public), [5727](https://github.com/FRC5727/SwervyBoi/tree/THOR2023) repositories
 */
public fun ChassisPowers.correctForDynamics(loopPeriod: Time = 0.02.seconds): ChassisPowers {
    val futureRobotPose = Pose2d(
        xPower * loopPeriod.inUnit(seconds),
        yPower * loopPeriod.inUnit(seconds),
        Rotation2d.fromRadians(rotationPower * loopPeriod.inUnit(seconds))
    )
    val twistForPose: Twist2d = log(futureRobotPose)
    return ChassisPowers(
        twistForPose.dx / loopPeriod.inUnit(seconds),
        twistForPose.dy / loopPeriod.inUnit(seconds),
        twistForPose.dtheta / loopPeriod.inUnit(seconds)
    )
}

