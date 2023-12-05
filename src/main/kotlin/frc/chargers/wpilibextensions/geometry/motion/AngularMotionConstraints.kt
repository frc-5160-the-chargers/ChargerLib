package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.trajectory.TrapezoidProfile

/**
 * A holder class that represents the constraints of a mechanism with angular acceleration/velocity,
 *  * with the respective [maxVelocity] and [maxAcceleration].
 *
 * @see TrapezoidProfile.Constraints
 */
public data class AngularMotionConstraints(
    val maxVelocity: AngularVelocity,
    val maxAcceleration: AngularAcceleration
){
    public fun inUnit(angleUnit: Angle, timeUnit: Time): TrapezoidProfile.Constraints =
        TrapezoidProfile.Constraints(
            maxVelocity.inUnit(angleUnit / timeUnit),
            maxAcceleration.inUnit(angleUnit / timeUnit / timeUnit)
        )
}