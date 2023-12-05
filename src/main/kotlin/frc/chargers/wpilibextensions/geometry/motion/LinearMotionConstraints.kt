package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.*
import com.pathplanner.lib.PathConstraints
import edu.wpi.first.math.trajectory.TrapezoidProfile

/**
 * A holder class that represents the constraints of a mechanism with linear acceleration/velocity,
 * with the respective [maxVelocity] and [maxAcceleration].
 *
 * @see TrapezoidProfile.Constraints
 */
public data class LinearMotionConstraints(
    val maxVelocity: Velocity,
    val maxAcceleration: Acceleration
){
    public fun inUnit(distanceUnit: Distance, timeUnit: Time): TrapezoidProfile.Constraints =
        TrapezoidProfile.Constraints(
            maxVelocity.inUnit(distanceUnit / timeUnit),
            maxAcceleration.inUnit(distanceUnit / timeUnit / timeUnit)
        )


    public fun toPathConstraints(): PathConstraints = PathConstraints(
        maxVelocity.siValue,
        maxAcceleration.siValue
    )
}