package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.asRotation2d

/**
 * A helper class that stores [SwerveModuleState]s in a more clear way.
 * This is usually preferred over an array, as it is clear which [SwerveModuleState] corresponds to which module.
 */
public data class ModuleSpeeds(
    var topLeftSpeed: Velocity,
    var topRightSpeed: Velocity,
    var bottomLeftSpeed: Velocity,
    var bottomRightSpeed: Velocity,
    
    var topLeftAngle: Angle,
    var topRightAngle: Angle,
    var bottomLeftAngle: Angle,
    var bottomRightAngle: Angle,
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

    public fun asDesaturatedSpeeds(maxAttainableSpeed: Velocity): ModuleSpeeds{
        // !! operator is 100% safe: list size is greater than 0.
        val maxSpeed = listOf(topLeftSpeed,topRightSpeed,bottomLeftSpeed,bottomRightSpeed).maxOrNull()!!
        return if (maxSpeed > maxAttainableSpeed){
            ModuleSpeeds(
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


public fun ModuleSpeeds.logAsDesiredSpeeds(){
    SmartDashboard.putNumber("top left DESIRED speed mps: ", topLeftSpeed.inUnit(meters/seconds))
    SmartDashboard.putNumber("top right DESIRED speed mps: ", topRightSpeed.inUnit(meters/seconds))
    SmartDashboard.putNumber("bottom left DESIRED speed mps: ", bottomLeftSpeed.inUnit(meters/seconds))
    SmartDashboard.putNumber("bottom right DESIRED speed mps: ", bottomRightSpeed.inUnit(meters/seconds))

    SmartDashboard.putNumber("top left DESIRED angle degrees: ", topLeftAngle.inUnit(degrees))
    SmartDashboard.putNumber("top right DESIRED angle degrees: ", topRightAngle.inUnit(degrees))
    SmartDashboard.putNumber("bottom left DESIRED angle degrees: ", bottomLeftAngle.inUnit(degrees))
    SmartDashboard.putNumber("bottom right DESIRED angle degrees: ", bottomRightAngle.inUnit(degrees))
}

public fun ModuleSpeeds.logAsCurrentSpeeds(){
    SmartDashboard.putNumber("top left current speed mps: ", topLeftSpeed.inUnit(meters/seconds))
    SmartDashboard.putNumber("top right current speed mps: ", topRightSpeed.inUnit(meters/seconds))
    SmartDashboard.putNumber("bottom left current speed mps: ", bottomLeftSpeed.inUnit(meters/seconds))
    SmartDashboard.putNumber("bottom right current speed mps: ", bottomRightSpeed.inUnit(meters/seconds))

    SmartDashboard.putNumber("top left current angle degrees: ", topLeftAngle.inUnit(degrees))
    SmartDashboard.putNumber("top right current angle degrees: ", topRightAngle.inUnit(degrees))
    SmartDashboard.putNumber("bottom left current angle degrees: ", bottomLeftAngle.inUnit(degrees))
    SmartDashboard.putNumber("bottom right current angle degrees: ", bottomRightAngle.inUnit(degrees))
}