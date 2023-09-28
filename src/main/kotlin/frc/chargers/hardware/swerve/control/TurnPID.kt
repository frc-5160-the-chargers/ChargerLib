package frc.chargers.hardware.swerve.control

import com.batterystaple.kmeasure.dimensions.AngleDimension
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile

// used to denote control type for turning PID of swerve
public sealed class TurnPID {

    public class Basic(
        public val pidConstants: PIDConstants,
        public val precision: Precision<AngleDimension> = Precision.AllowOvershoot
    ): TurnPID()
    public class Profiled(
        public val pidConstants: PIDConstants,
        public val constraints: AngularTrapezoidProfile.Constraints,
        public val turnFF: AngularMotorFF = AngularMotorFF.None,
        public val precision: Precision<AngleDimension> = Precision.AllowOvershoot
    ): TurnPID()
}