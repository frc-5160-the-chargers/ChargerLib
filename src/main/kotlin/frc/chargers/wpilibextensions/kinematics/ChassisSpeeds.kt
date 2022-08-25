package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.ChassisSpeeds

public fun ChassisSpeeds(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): ChassisSpeeds =
    ChassisSpeeds(
        /* vxMetersPerSecond = */ xVelocity.inUnit(meters/seconds),
        /* vyMetersPerSecond = */ yVelocity.inUnit(meters/seconds),
        /* omegaRadiansPerSecond = */ rotationSpeed.inUnit(radians/seconds)
    )