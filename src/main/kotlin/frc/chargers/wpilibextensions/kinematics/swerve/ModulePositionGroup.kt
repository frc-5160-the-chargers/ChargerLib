package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.kinematics.SwerveModulePosition
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.asRotation2d

/**
 * A helper class that stores [SwerveModulePosition]s in a more clear way.
 * This is usually preferred over an array, as it is clear which [SwerveModulePosition] corresponds to which module.
 */
public data class ModulePositionGroup(
    val topLeftDistance: Distance,
    val topRightDistance: Distance,
    val bottomLeftDistance: Distance,
    val bottomRightDistance: Distance,

    val topLeftAngle: Angle,
    val topRightAngle: Angle,
    val bottomLeftAngle: Angle,
    val bottomRightAngle: Angle
) {
    public constructor(
        topLeftPosition: SwerveModulePosition,
        topRightPosition: SwerveModulePosition,
        bottomLeftPosition: SwerveModulePosition,
        bottomRightPosition: SwerveModulePosition,
    ): this(
        topLeftPosition.distance,
        topRightPosition.distance,
        bottomLeftPosition.distance,
        bottomRightPosition.distance,

        topLeftPosition.direction,
        topRightPosition.direction,
        bottomLeftPosition.direction,
        bottomRightPosition.direction
    )

    public fun toArray(): Array<SwerveModulePosition> = a[topLeftPosition,topRightPosition,bottomLeftPosition,bottomRightPosition]

    public val topLeftPosition: SwerveModulePosition = SwerveModulePosition(
        topLeftDistance.inUnit(meters),
        topLeftAngle.asRotation2d()
    )

    public val topRightPosition: SwerveModulePosition = SwerveModulePosition(
        topRightDistance.inUnit(meters),
        topRightAngle.asRotation2d()
    )

    public val bottomLeftPosition: SwerveModulePosition = SwerveModulePosition(
        bottomLeftDistance.inUnit(meters),
        bottomLeftAngle.asRotation2d()
    )

    public val bottomRightPosition: SwerveModulePosition = SwerveModulePosition(
        bottomRightDistance.inUnit(meters),
        bottomRightAngle.asRotation2d()
    )
}