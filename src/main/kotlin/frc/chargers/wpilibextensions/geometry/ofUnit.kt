package frc.chargers.wpilibextensions.geometry


import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile

/**
 * Adaptors that convert some of WPILib's geometry-related classes to the Chargerlib Wrappers.
 */

/**
 * Converts WPILib's [Pose2d] into a [UnitPose2d].
 * @see Pose2d
 * @see UnitPose2d
 */
public fun Pose2d.ofUnit(unit: Distance): UnitPose2d = UnitPose2d(this.x.ofUnit(unit),this.y.ofUnit(unit),this.rotation.asAngle())

/**
 * Converts WPILib's [Translation2d] into a [UnitTranslation2d].
 * @see Translation2d
 * @see UnitTranslation2d
 */
public fun Translation2d.ofUnit(unit: Distance): UnitTranslation2d = UnitTranslation2d(this.norm.ofUnit(unit),this.angle.asAngle())
public fun Transform2d.ofUnit(unit: Distance): UnitTransform2d = UnitTransform2d(this.translation.ofUnit(unit),this.rotation.asAngle())

public fun TrapezoidProfile.State.ofUnit(angleUnit: Angle, timeUnit: Time): AngularTrapezoidProfile.State =
    AngularTrapezoidProfile.State(
        this.position.ofUnit(angleUnit),
        this.velocity.ofUnit(angleUnit/timeUnit)
    )

public fun TrapezoidProfile.Constraints.ofUnit(angleUnit: Angle, timeUnit: Time): AngularTrapezoidProfile.Constraints =
    AngularTrapezoidProfile.Constraints(
        this.maxVelocity.ofUnit(angleUnit/timeUnit),
        this.maxAcceleration.ofUnit(angleUnit/timeUnit/timeUnit)
    )

public fun TrapezoidProfile.State.ofUnit(distanceUnit: Distance, timeUnit: Time): LinearTrapezoidProfile.State =
    LinearTrapezoidProfile.State(
        this.position.ofUnit(distanceUnit),
        this.velocity.ofUnit(distanceUnit/timeUnit)
    )

public fun TrapezoidProfile.Constraints.ofUnit(distanceUnit: Distance, timeUnit: Time): LinearTrapezoidProfile.Constraints =
    LinearTrapezoidProfile.Constraints(
        this.maxVelocity.ofUnit(distanceUnit/timeUnit),
        this.maxAcceleration.ofUnit(distanceUnit/timeUnit/timeUnit)
    )