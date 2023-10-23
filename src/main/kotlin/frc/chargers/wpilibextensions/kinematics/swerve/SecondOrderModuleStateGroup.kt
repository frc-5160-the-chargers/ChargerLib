package frc.chargers.wpilibextensions.kinematics.swerve

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Velocity
import edu.wpi.first.math.kinematics.SwerveModuleState

public class SecondOrderModuleStateGroup(
    topLeftSpeed: Velocity = Velocity(0.0),
    topRightSpeed: Velocity = Velocity(0.0),
    bottomLeftSpeed: Velocity = Velocity(0.0),
    bottomRightSpeed: Velocity = Velocity(0.0),

    topLeftAngle: Angle = Angle(0.0),
    topRightAngle: Angle = Angle(0.0),
    bottomLeftAngle: Angle = Angle(0.0),
    bottomRightAngle: Angle = Angle(0.0),
            
    public var topLeftTurnSpeed: AngularVelocity = AngularVelocity(0.0),
    public var topRightTurnSpeed: AngularVelocity = AngularVelocity(0.0),
    public var bottomLeftTurnSpeed: AngularVelocity = AngularVelocity(0.0),
    public var bottomRightTurnSpeed: AngularVelocity = AngularVelocity(0.0)
): ModuleStateGroup(topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed, topLeftAngle, topRightAngle, bottomLeftAngle, bottomRightAngle) {

    public constructor(
        topLeftState: SwerveModuleState,
        topRightState: SwerveModuleState,
        bottomLeftState: SwerveModuleState,
        bottomRightState: SwerveModuleState,
        topLeftTurnSpeed: AngularVelocity = AngularVelocity(0.0),
        topRightTurnSpeed: AngularVelocity = AngularVelocity(0.0),
        bottomLeftTurnSpeed: AngularVelocity = AngularVelocity(0.0),
        bottomRightTurnSpeed: AngularVelocity = AngularVelocity(0.0)
    ): this(
        topLeftState.speed,
        topRightState.speed,
        bottomLeftState.speed,
        bottomRightState.speed,

        topLeftState.direction,
        topRightState.direction,
        bottomLeftState.direction,
        bottomRightState.direction,

        topLeftTurnSpeed, topRightTurnSpeed, bottomLeftTurnSpeed, bottomRightTurnSpeed
    )



}