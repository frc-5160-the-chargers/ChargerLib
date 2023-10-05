package frc.chargers.hardware.swerve.control

import com.batterystaple.kmeasure.dimensions.AngleDimension
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile

/**
 * A Helper class used to denote the direction control type of a [frc.chargers.hardware.swerve.module.SwerveModule]: Either
 * Basic PID(regular PID controller) or ProfiledPID(motion profile + PID control, with optional feedforward).
 */
public sealed class SwerveAngleControl {

    public class PID(
        public val pidConstants: PIDConstants,
        public val precision: Precision<AngleDimension> = Precision.AllowOvershoot
    ): SwerveAngleControl()
    public class ProfiledPID(
        public val pidConstants: PIDConstants,
        public val constraints: AngularTrapezoidProfile.Constraints,
        public val turnFF: AngularMotorFF = AngularMotorFF.None,
        public val precision: Precision<AngleDimension> = Precision.AllowOvershoot
    ): SwerveAngleControl()
}