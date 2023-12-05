package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.Velocity
import frc.chargers.constants.MK4i
import frc.chargers.utils.math.units.Inertia

/**
 * A class used to hold constants for an [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 */
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
        /**
         * Creates a [SwerveConstants] instance with auto-completed constants
         * related to Mk4i L2 swerve modules.
         */
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

        /**
         * Creates a [SwerveConstants] instance with auto-completed constants
         * related to Mk4i L3 swerve modules.
         */
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

        /**
         * Creates a [SwerveConstants] instance with auto-completed constants
         * related to Mk4i L1 swerve modules.
         */
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