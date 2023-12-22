package frc.chargers.pathplannerextensions


import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import frc.chargers.wpilibextensions.geometry.rotation.asRotation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d


public fun bezierFromPoses(vararg poses: UnitPose2d): List<UnitTranslation2d> =
    PathPlannerPath.bezierFromPoses(poses.map{it.siValue}).map{UnitTranslation2d(it)}

public fun PathConstraints(
    maxLinearVelocity: Velocity,
    maxLinearAcceleration: Acceleration,
    maxAngularVelocity: AngularVelocity,
    maxAngularAcceleration: AngularAcceleration
): PathConstraints = PathConstraints(
    maxLinearVelocity.inUnit(meters/seconds),
    maxLinearAcceleration.inUnit(meters/seconds/seconds),
    maxAngularVelocity.inUnit(radians/seconds),
    maxAngularAcceleration.inUnit(radians/seconds/seconds)
)


public fun GoalEndState(
    velocity: Velocity,
    rotation: Angle
): GoalEndState = GoalEndState(
    velocity.inUnit(meters/seconds), rotation.asRotation2d()
)


public fun PathPlannerPath(
    bezierPoints: List<UnitTranslation2d>,
    constraints: PathConstraints,
    endState: GoalEndState,
): PathPlannerPath = PathPlannerPath(
    bezierPoints.map{it.siValue},
    constraints,endState
)

