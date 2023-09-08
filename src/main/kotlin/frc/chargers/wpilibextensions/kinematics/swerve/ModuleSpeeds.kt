package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.asRotation2d

public data class ModuleSpeeds(
    val topLeftSpeed: Velocity,
    val topRightSpeed: Velocity,
    val bottomLeftSpeed: Velocity,
    val bottomRightSpeed: Velocity,
    
    val topLeftAngle: Angle,
    val topRightAngle: Angle,
    val bottomLeftAngle: Angle,
    val bottomRightAngle: Angle
) {
    public constructor(
        topLeftState: SwerveModuleState,
        topRightState: SwerveModuleState,
        bottomLeftState: SwerveModuleState,
        bottomRightState: SwerveModuleState,
    ): this(
        topLeftState.speed,
        topRightState.speed,
        bottomLeftState.speed,
        bottomRightState.speed,
        
        topLeftState.direction,
        topRightState.direction,
        bottomLeftState.direction,
        bottomRightState.direction
    )

    public fun toArray(): Array<SwerveModuleState> = a[topLeftState,topRightState,bottomLeftState,bottomRightState]
    
    public val topLeftState: SwerveModuleState
        get() = SwerveModuleState(
        topLeftSpeed.inUnit(meters/seconds),
        topLeftAngle.asRotation2d()
    )

    public val topRightState: SwerveModuleState
        get() = SwerveModuleState(
        topRightSpeed.inUnit(meters/seconds),
        topRightAngle.asRotation2d()
    )

    public val bottomLeftState: SwerveModuleState
        get() = SwerveModuleState(
        bottomLeftSpeed.inUnit(meters/seconds),
        bottomLeftAngle.asRotation2d()
    )

    public val bottomRightState: SwerveModuleState
        get() = SwerveModuleState(
        bottomRightSpeed.inUnit(meters/seconds),
        bottomRightAngle.asRotation2d()
    )
}