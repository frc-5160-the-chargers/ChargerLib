package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.kinematics.SwerveModulePosition
import frc.chargers.wpilibextensions.geometry.asAngle
import frc.chargers.wpilibextensions.geometry.asRotation2d

public fun SwerveModulePosition(
    distance: Distance,
    direction: Angle
): SwerveModulePosition = SwerveModulePosition(
    distance.inUnit(meters),
    direction.asRotation2d()
)

public val SwerveModulePosition.distance: Distance
    get() = distanceMeters.ofUnit(meters)

public val SwerveModulePosition.direction: Angle
    get() = angle.asAngle()