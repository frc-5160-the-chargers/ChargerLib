package frc.chargers.pathplannerextensions

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.PathConstraints
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints

public fun PathConstraints(
    maxLinearVelocity: Velocity,
    maxLinearAcceleration: Acceleration,
    maxAngularVelocity: AngularVelocity,
    maxAngularAcceleration: AngularAcceleration
): PathConstraints = PathConstraints(
    maxLinearVelocity.inUnit(meters / seconds),
    maxLinearAcceleration.inUnit(meters / seconds / seconds),
    maxAngularVelocity.inUnit(radians / seconds),
    maxAngularAcceleration.inUnit(radians / seconds / seconds)
)

public fun PathConstraints(
    linearConstraints: LinearMotionConstraints,
    angularConstraints: AngularMotionConstraints
): PathConstraints = PathConstraints(
    linearConstraints.maxVelocity.siValue,
    linearConstraints.maxAcceleration.siValue,
    angularConstraints.maxVelocity.siValue,
    angularConstraints.maxAcceleration.siValue
)