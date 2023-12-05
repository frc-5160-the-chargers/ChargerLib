package frc.chargers.hardware.subsystemutils.swervedrive

import com.batterystaple.kmeasure.dimensions.AngleDimension
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints


/**
 * A Helper class that hold parameters needed to control an
 * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 *
 * @see SwerveControl.PIDFirstOrder
 * @see SwerveControl.PIDSecondOrder
 * @see SwerveControl.ProfiledPIDFirstOrder
 * @see SwerveControl.ProfiledPIDSecondOrder
 */
public sealed class SwerveControl(
    public val turnPIDConstants: PIDConstants,
    public val turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    public val drivePIDConstants: PIDConstants,
    public val driveFF: AngularMotorFF,
    public val staticVoltageStall: Boolean = false
) {

    /**
     * Holds constants for pure PID control + first order kinematics for an
     * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
     */
    public class PIDFirstOrder(
        turnPIDConstants: PIDConstants,
        turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        drivePIDConstants: PIDConstants,
        driveFF: AngularMotorFF,
        staticVoltageStall: Boolean = false
    ): SwerveControl(turnPIDConstants, turnPrecision, drivePIDConstants, driveFF, staticVoltageStall)

    /**
     * Holds constants for pure PID control + second order kinematics for an
     * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
     *
     * Here, a feedforward is necessary to process the ModuleTurnSpeeds output of the second kinematics.
     */
    public class PIDSecondOrder(
        turnPIDConstants: PIDConstants,
        turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        override val turnFF: AngularMotorFF,
        drivePIDConstants: PIDConstants,
        driveFF: AngularMotorFF,
        staticVoltageStall: Boolean = false
    ): SwerveControl(turnPIDConstants, turnPrecision, drivePIDConstants, driveFF, staticVoltageStall),
        SecondOrderControlScheme


    /**
     * Holds constants for Profiled PID control + first order kinematics for an
     * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
     */
    public class ProfiledPIDFirstOrder(
        turnPIDConstants: PIDConstants,
        turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        override val turnConstraints: AngularMotionConstraints,
        override val turnFF: AngularMotorFF = AngularMotorFF.None,
        drivePIDConstants: PIDConstants,
        driveFF: AngularMotorFF,
        staticVoltageStall: Boolean = false
    ): SwerveControl(turnPIDConstants, turnPrecision, drivePIDConstants, driveFF, staticVoltageStall),
        ProfiledPIDControlScheme

    /**
     * Holds constants for Profiled PID control + second order kinematics for an
     * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
     */
    public class ProfiledPIDSecondOrder(
        turnPIDConstants: PIDConstants,
        turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        override val turnConstraints: AngularMotionConstraints,
        override val turnFF: AngularMotorFF,
        drivePIDConstants: PIDConstants,
        driveFF: AngularMotorFF,
        staticVoltageStall: Boolean = false
    ): SwerveControl(turnPIDConstants, turnPrecision, drivePIDConstants, driveFF, staticVoltageStall),
        SecondOrderControlScheme, ProfiledPIDControlScheme

}


/**
 * Represents a generic object that holds a feedforward necessary for processing ModuleTurnSpeeds.
 */
public interface SecondOrderControlScheme{
    public val turnFF: AngularMotorFF
}

/**
 * Represents a generic object that holds the Trapezoidal constraints necessary for Profiled PID control.
 */
public interface ProfiledPIDControlScheme{
    public val turnConstraints: AngularMotionConstraints
    public val turnFF: AngularMotorFF
        get() = AngularMotorFF.None
}
