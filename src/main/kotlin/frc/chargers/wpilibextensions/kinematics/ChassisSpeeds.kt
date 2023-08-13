package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.wpilibextensions.geometry.asRotation2d


public object ChassisSpeeds{

}

public fun ChassisSpeeds(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): ChassisSpeeds =
    ChassisSpeeds(
        /* vxMetersPerSecond = */ xVelocity.inUnit(meters/seconds),
        /* vyMetersPerSecond = */ yVelocity.inUnit(meters/seconds),
        /* omegaRadiansPerSecond = */ rotationSpeed.inUnit(radians/seconds)
    )

public fun FieldRelativeChassisSpeeds(speeds: ChassisSpeeds, robotAngle: Angle): ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    speeds,
    robotAngle.asRotation2d()
)

public fun FieldRelativeChassisSpeeds(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity, robotAngle: Angle): ChassisSpeeds =
    FieldRelativeChassisSpeeds(
        ChassisSpeeds(xVelocity,yVelocity,rotationSpeed),
        robotAngle
    )

public val ChassisSpeeds.xVelocity: Velocity
    get() = vxMetersPerSecond.ofUnit(meters/seconds)
public val ChassisSpeeds.yVelocity: Velocity
    get() = vyMetersPerSecond.ofUnit(meters/seconds)
public val ChassisSpeeds.rotationSpeed: AngularVelocity
    get() = omegaRadiansPerSecond.ofUnit(radians/seconds)






