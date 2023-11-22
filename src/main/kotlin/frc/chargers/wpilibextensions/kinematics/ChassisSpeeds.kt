package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.framework.ChargerRobot


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


/**
 * A function used to correct for drift on swerve drive when simultaneously rotating and driving in a singular direction.
 *
 * Credits: [254](https://github.com/Team254/FRC-2022-Public), [5727](https://github.com/FRC5727/SwervyBoi/tree/THOR2023) repositories
 */
public fun ChassisSpeeds.correctForDynamics(loopPeriod: Time? = null, multiplier: Double = 1.0): ChassisSpeeds {

    val period = loopPeriod ?: ChargerRobot.LOOP_PERIOD

    val futureRobotPose = Pose2d(
        vxMetersPerSecond * period.inUnit(seconds),
        vyMetersPerSecond * period.inUnit(seconds),
        Rotation2d.fromRadians(omegaRadiansPerSecond * period.inUnit(seconds) * multiplier)
    )
    val twistForPose: Twist2d = Pose2d().log(futureRobotPose)
    return ChassisSpeeds(
        twistForPose.dx / period.inUnit(seconds),
        twistForPose.dy / period.inUnit(seconds),
        this.omegaRadiansPerSecond
    )
}







