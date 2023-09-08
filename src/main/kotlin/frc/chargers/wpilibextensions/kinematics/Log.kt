package frc.chargers.wpilibextensions.kinematics

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

private const val kEps = 1E-9
/**
 * Credits: [5727](https://github.com/FRC5727/SwervyBoi/tree/THOR2023)
 *
 * Used for ChassisPowers/ChassisSpeeds correction
 */
public fun log(transform: Pose2d): Twist2d {
    val dtheta = transform.rotation.radians
    val half_dtheta = 0.5 * dtheta
    val cos_minus_one = cos(transform.rotation.radians) - 1.0
    val halftheta_by_tan_of_halfdtheta: Double = if (abs(cos_minus_one) < kEps) {
        1.0 - 1.0 / 12.0 * dtheta * dtheta
    } else {
        -(half_dtheta * sin(transform.rotation.radians)) / cos_minus_one
    }
    val translation_part = transform
        .translation
        .rotateBy(Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta))
    return Twist2d(translation_part.x, translation_part.y, dtheta)
}