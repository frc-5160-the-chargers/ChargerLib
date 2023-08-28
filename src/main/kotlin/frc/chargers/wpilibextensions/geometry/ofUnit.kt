package frc.chargers.wpilibextensions.geometry


import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.trajectory.TrapezoidProfile

/**
 * Adaptor functions that convert some of WPILib's geometry-related classes to the Chargerlib Wrappers.
 */

/**
 * Converts WPILib's [Pose2d] into a [UnitPose2d].
 */
public fun Pose2d.ofUnit(unit: Distance): UnitPose2d = UnitPose2d(x.ofUnit(unit),y.ofUnit(unit),rotation.asAngle())

/**
 * Converts WPILib's [Translation2d] into a [UnitTranslation2d].
 */
public fun Translation2d.ofUnit(unit: Distance): UnitTranslation2d = UnitTranslation2d(norm.ofUnit(unit),angle.asAngle())

/**
 * Converts WPILib's [Transform2d] into a [UnitTransform2d].
 */
public fun Transform2d.ofUnit(unit: Distance): UnitTransform2d = UnitTransform2d(translation.ofUnit(unit),rotation.asAngle())

/*
public fun Translation3d.ofUnit(unit: Distance): UnitTranslation3d = UnitTranslation3d(
    x.ofUnit(unit),
    y.ofUnit(unit),
    z.ofUnit(unit)
)
 */

/**
 * Converts WPILib's [TrapezoidProfile.State] into an [AngularTrapezoidProfile.State].
 */
public fun TrapezoidProfile.State.ofUnit(angleUnit: Angle, timeUnit: Time): AngularTrapezoidProfile.State =
    AngularTrapezoidProfile.State(
        this.position.ofUnit(angleUnit),
        this.velocity.ofUnit(angleUnit/timeUnit)
    )

/**
 * Converts WPILib's [TrapezoidProfile.Constraints] into an [AngularTrapezoidProfile.Constraints].
 */
public fun TrapezoidProfile.Constraints.ofUnit(angleUnit: Angle, timeUnit: Time): AngularTrapezoidProfile.Constraints =
    AngularTrapezoidProfile.Constraints(
        this.maxVelocity.ofUnit(angleUnit/timeUnit),
        this.maxAcceleration.ofUnit(angleUnit/timeUnit/timeUnit)
    )
/**
 * Converts WPILib's [TrapezoidProfile.State] into a [LinearTrapezoidProfile.State].
 */
public fun TrapezoidProfile.State.ofUnit(distanceUnit: Distance, timeUnit: Time): LinearTrapezoidProfile.State =
    LinearTrapezoidProfile.State(
        this.position.ofUnit(distanceUnit),
        this.velocity.ofUnit(distanceUnit/timeUnit)
    )

/**
 * Converts WPILib's [TrapezoidProfile.Constraints] into a [LinearTrapezoidProfile.Constraints].
 */
public fun TrapezoidProfile.Constraints.ofUnit(distanceUnit: Distance, timeUnit: Time): LinearTrapezoidProfile.Constraints =
    LinearTrapezoidProfile.Constraints(
        this.maxVelocity.ofUnit(distanceUnit/timeUnit),
        this.maxAcceleration.ofUnit(distanceUnit/timeUnit/timeUnit)
    )