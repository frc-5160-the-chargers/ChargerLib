package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.inches
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.utils.math.units.Inertia

/**
 * A class that holds Control data for a [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 * This includes PID constants, feedforward, and path replanning configs.
 */
public data class SwerveControlData(
    val anglePID: PIDConstants,
    val angleSetpointSupplier: SetpointSupplier<AngleDimension, VoltageDimension> = SetpointSupplier.Default(),
    val modulePrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFFConstants,
    val robotRotationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    val robotTranslationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    val pathReplanConfig: ReplanningConfig = ReplanningConfig()
)

/**
 * A class that holds Hardware constants for a [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 * This includes trackwidth, wheelbase, inertia, turn motor inversion, and max speed.
 */
public data class SwerveHardwareData(
    val invertTurnMotors: Boolean = false,
    val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    val turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    val driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val wheelDiameter: Length,
    val trackWidth: Distance,
    val wheelBase: Distance
){
    public companion object{
        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to Mk4i L2 swerve modules.
         */
        public fun mk4iL2(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance,
            turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            7.0 / 150.0,
            1.0 / 6.75,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to Mk4i L3 swerve modules.
         */
        public fun mk4iL3(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance,
            turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            7.0 / 150.0,
            1.0 / 6.12,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to Mk4i L1 swerve modules.
         */
        public fun mk4iL1(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA,
            trackWidth: Distance,
            wheelBase: Distance
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            7.0 / 150.0,
            1.0 / 8.14,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )
    }

}