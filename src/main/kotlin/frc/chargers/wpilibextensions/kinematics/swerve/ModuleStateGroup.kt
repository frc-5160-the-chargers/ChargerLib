package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.asRotation2d

/**
 * A helper class that stores [SwerveModuleState]s in a more clear way.
 * This is usually preferred over an array, as it is clear which [SwerveModuleState] corresponds to which module.
 */
public data class ModuleStateGroup(
    var topLeftSpeed: Velocity = Velocity(0.0),
    var topRightSpeed: Velocity = Velocity(0.0),
    var bottomLeftSpeed: Velocity = Velocity(0.0),
    var bottomRightSpeed: Velocity = Velocity(0.0),
    
    var topLeftAngle: Angle = Angle(0.0),
    var topRightAngle: Angle = Angle(0.0),
    var bottomLeftAngle: Angle = Angle(0.0),
    var bottomRightAngle: Angle = Angle(0.0),
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

    public fun desaturate(maxAttainableSpeed: Velocity){
        // !! operator is 100% safe: list size is greater than 0.
        val maxSpeed = listOf(topLeftSpeed,topRightSpeed,bottomLeftSpeed,bottomRightSpeed).maxOrNull()!!
        if (maxSpeed > maxAttainableSpeed){
            topLeftSpeed = topLeftSpeed / maxSpeed * maxAttainableSpeed
            topRightSpeed = topRightSpeed / maxSpeed * maxAttainableSpeed
            bottomLeftSpeed = bottomLeftSpeed / maxSpeed * maxAttainableSpeed
            bottomRightSpeed = bottomRightSpeed / maxSpeed * maxAttainableSpeed
        }
    }

    public fun asDesaturatedSpeeds(maxAttainableSpeed: Velocity): ModuleStateGroup{
        // !! operator is 100% safe: list size is greater than 0.
        val maxSpeed = listOf(topLeftSpeed,topRightSpeed,bottomLeftSpeed,bottomRightSpeed).maxOrNull()!!
        return if (maxSpeed > maxAttainableSpeed){
            ModuleStateGroup(
                topLeftSpeed / maxSpeed * maxAttainableSpeed,
                topRightSpeed / maxSpeed * maxAttainableSpeed,
                bottomLeftSpeed / maxSpeed * maxAttainableSpeed,
                bottomRightSpeed / maxSpeed * maxAttainableSpeed,
                topLeftAngle,
                topRightAngle,
                bottomLeftAngle,
                bottomRightAngle
            )
        }else{
            this
        }
    }

    public inline fun forEachState(action: (SwerveModuleState) -> Unit){
        action(topLeftState)
        action(topRightState)
        action(bottomLeftState)
        action(bottomRightState)
    }
    
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

