package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.wpilibextensions.geometry.asAngle
import frc.chargers.wpilibextensions.geometry.asRotation2d

public fun SwerveModuleState(
    speed: Velocity,
    direction: Angle
): SwerveModuleState = SwerveModuleState(
    speed.inUnit(meters/seconds),
    direction.asRotation2d()
)
public val SwerveModuleState.speed: Velocity
    get() = speedMetersPerSecond.ofUnit(meters/seconds)

public val SwerveModuleState.direction: Angle
    get() = angle.asAngle()