package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.wpilibextensions.geometry.asRotation2d


/**
 * A convenience function that creates a [ChassisSpeeds]
 *
 * with kmeasure units instead.
 */
public fun ChassisSpeeds(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): ChassisSpeeds =
    ChassisSpeeds(
        /* vxMetersPerSecond = */ xVelocity.inUnit(meters/seconds),
        /* vyMetersPerSecond = */ yVelocity.inUnit(meters/seconds),
        /* omegaRadiansPerSecond = */ rotationSpeed.inUnit(radians/seconds)
    )


public val ChassisSpeeds.xVelocity: Velocity
    get() = vxMetersPerSecond.ofUnit(meters/seconds)
public val ChassisSpeeds.yVelocity: Velocity
    get() = vyMetersPerSecond.ofUnit(meters/seconds)
public val ChassisSpeeds.rotationSpeed: AngularVelocity
    get() = omegaRadiansPerSecond.ofUnit(radians/seconds)

private fun log(transform: Pose2d): Twist2d {
    val dtheta = transform.rotation.radians
    val half_dtheta = 0.5 * dtheta
    val cos_minus_one = kotlin.math.cos(transform.rotation.radians) - 1.0
    val halftheta_by_tan_of_halfdtheta: Double = if (kotlin.math.abs(cos_minus_one) < 1E-9) {
        1.0 - 1.0 / 12.0 * dtheta * dtheta
    } else {
        -(half_dtheta * kotlin.math.sin(transform.rotation.radians)) / cos_minus_one
    }
    val translation_part = transform.translation.rotateBy(Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta))
    return Twist2d(translation_part.x, translation_part.y, dtheta)
}

/**
 * A function used to correct for drift on swerve drive when simultaneously rotating and driving in a singular direction.
 *
 * Credits: [254](https://github.com/Team254/FRC-2022-Public), [5727](https://github.com/FRC5727/SwervyBoi/tree/THOR2023) repositories
 */
public fun ChassisSpeeds.correctForDynamics(loopPeriod: Time = 0.02.seconds): ChassisSpeeds {
    val futureRobotPose = Pose2d(
        vxMetersPerSecond * loopPeriod.inUnit(seconds),
        vyMetersPerSecond * loopPeriod.inUnit(seconds),
        Rotation2d.fromRadians(omegaRadiansPerSecond * loopPeriod.inUnit(seconds))
    )
    val twistForPose: Twist2d = log(futureRobotPose)
    return ChassisSpeeds(
        twistForPose.dx / loopPeriod.inUnit(seconds),
        twistForPose.dy / loopPeriod.inUnit(seconds),
        twistForPose.dtheta / loopPeriod.inUnit(seconds)
    )
}







