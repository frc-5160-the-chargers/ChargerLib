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

/**
 * A class that holds Control data for a [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain].
 * This includes PID constants, feedforward, and path re-planning configs.
 */
public data class SwerveControlData(
    val anglePID: PIDConstants,
    val angleSetpointSupplier: SetpointSupplier<AngleDimension, VoltageDimension> = SetpointSupplier.Default(),
    val modulePrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFFConstants,
    val openLoopDiscretizationRate: Double = 2.0,
    val closedLoopDiscretizationRate: Double = 1.0,
    val robotRotationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    val robotTranslationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    val pathReplanConfig: ReplanningConfig = ReplanningConfig()
)

/**
 * A class that holds Hardware constants for a [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain].
 * This includes Track width, wheelbase, inertia, turn motor inversion, and max speed.
 */
public data class SwerveHardwareData(
    val invertTurnMotors: Boolean = false,
    val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    val turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
    val driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val wheelDiameter: Length,
    val trackWidth: Distance,
    val wheelBase: Distance
){
    public companion object{
        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L2 swerve modules.
         */
        public fun mk4iL2(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            150.0 / 7.0,
            6.75,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L3 swerve modules.
         */
        public fun mk4iL3(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            trackWidth: Distance,
            wheelBase: Distance,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            150.0 / 7.0,
            6.12,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L1 swerve modules.
         */
        public fun mk4iL1(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
            trackWidth: Distance,
            wheelBase: Distance
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            150.0 / 7.0,
            8.14,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )
    }

}