package frc.chargers.wpilibextensions.geometry


import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.utils.math.units.KmeasureUnit
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTranslation3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTransform2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d

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


/**
 * Converts WPILib's [Translation3d] into a [UnitTranslation3d].
 */
public fun Translation3d.ofUnit(unit: Distance): UnitTranslation3d = UnitTranslation3d(
    x.ofUnit(unit),
    y.ofUnit(unit),
    z.ofUnit(unit)
)

/**
 * Converts WPILib's [Transform3d] into a [UnitTransform3d].
 */
public fun Transform3d.ofUnit(unit: KmeasureUnit<DistanceDimension>): UnitTransform3d =
    UnitTransform3d(
        translation.ofUnit(unit),
        rotation
    )

/**
 * Converts WPILib's [Pose3d] into a [UnitPose3d].
 */
public fun Pose3d.ofUnit(unit: KmeasureUnit<DistanceDimension>): UnitPose3d =
    UnitPose3d(
        x.ofUnit(unit),
        y.ofUnit(unit),
        z.ofUnit(unit),
        rotation
    )


/**
 * Converts WPILib's [TrapezoidProfile.Constraints] into an [AngularMotionConstraints].
 */
public fun TrapezoidProfile.Constraints.ofUnit(angleUnit: Angle, timeUnit: Time): AngularMotionConstraints =
    AngularMotionConstraints(
        this.maxVelocity.ofUnit(angleUnit/timeUnit),
        this.maxAcceleration.ofUnit(angleUnit/timeUnit/timeUnit)
    )

/**
 * Converts WPILib's [TrapezoidProfile.Constraints] into a [LinearMotionConstraints].
 */
public fun TrapezoidProfile.Constraints.ofUnit(distanceUnit: Distance, timeUnit: Time): LinearMotionConstraints =
    LinearMotionConstraints(
        this.maxVelocity.ofUnit(distanceUnit/timeUnit),
        this.maxAcceleration.ofUnit(distanceUnit/timeUnit/timeUnit)
    )