// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * A helper class that stores direction powers for drivetrain classes.
 */
public class ChassisPowers(
    public var xPower: Double = 0.0,
    public var yPower: Double = 0.0,
    public var rotationPower: Double = 0.0
){
    public fun toChassisSpeeds(maxLinearVelocity: Velocity, maxRotationalVelocity: AngularVelocity): ChassisSpeeds = ChassisSpeeds(
        xPower * maxLinearVelocity,
        yPower * maxLinearVelocity,
        rotationPower * maxRotationalVelocity
    )
}





