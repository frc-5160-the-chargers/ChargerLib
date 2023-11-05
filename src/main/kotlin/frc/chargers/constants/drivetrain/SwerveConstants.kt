package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.Velocity
import frc.chargers.constants.MK4i
import frc.chargers.hardware.subsystems.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.hardware.subsystems.drivetrain.DEFAULT_MAX_MODULE_SPEED
import frc.chargers.hardware.subsystemutils.swervedrive.module.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.hardware.subsystemutils.swervedrive.module.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.utils.math.units.Inertia

public data class SwerveConstants(
    val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    val turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    val driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val wheelDiameter: Length,
    val trackWidth: Distance,
    val wheelBase: Distance,
){
    public companion object{
        public fun mk4iL2(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance,
            turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveConstants = SwerveConstants(
            MK4i.TURN_GEAR_RATIO,
            MK4i.GEAR_RATIO_L2,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            MK4i.WHEEL_DIAMETER,
            trackWidth,
            wheelBase
        )

        public fun mk4iL3(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance,
            turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveConstants = SwerveConstants(
            MK4i.TURN_GEAR_RATIO,
            MK4i.GEAR_RATIO_L3,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            MK4i.WHEEL_DIAMETER,
            trackWidth,
            wheelBase
        )

        public fun mk4iL1(
            turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance
        ): SwerveConstants = SwerveConstants(
            MK4i.TURN_GEAR_RATIO,
            MK4i.GEAR_RATIO_L1,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            MK4i.WHEEL_DIAMETER,
            trackWidth,
            wheelBase
        )
    }

}