package frc.chargers.hardware.subsystemutils.swervedrive

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints


/**
 * A Helper class that hold parameters needed to control an
 * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 *
 * @see SwerveControl.PID
 * @see SwerveControl.ProfiledPID
 */
public sealed class SwerveControl(
    public val anglePID: PIDConstants,
    public val modulePrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    public val velocityPID: PIDConstants,
    public val velocityFF: AngularMotorFF,
    public val robotRotationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    public val robotTranslationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    public val pathReplanConfig: ReplanningConfig = ReplanningConfig()
) {
    /**
     * Holds constants for pure PID control + first order kinematics for an
     * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
     */
    public class PID(
        anglePID: PIDConstants,
        modulePrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        velocityPID: PIDConstants,
        velocityFF: AngularMotorFF,
        robotRotationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
        robotTranslationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
        pathReplanConfig: ReplanningConfig = ReplanningConfig()
    ): SwerveControl(anglePID, modulePrecision, velocityPID, velocityFF, robotRotationPID, robotTranslationPID, pathReplanConfig)


    /**
     * Holds constants for Profiled PID control + first order kinematics for an
     * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
     */
    public class ProfiledPID(
        anglePID: PIDConstants,
        modulePrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        public val angleTargetConstraints: AngularMotionConstraints,
        public val angleTargetFF: AngularMotorFF = AngularMotorFF.None,
        velocityPID: PIDConstants,
        velocityFF: AngularMotorFF,
        robotRotationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
        robotTranslationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
        pathReplanConfig: ReplanningConfig = ReplanningConfig()
    ): SwerveControl(anglePID, modulePrecision, velocityPID, velocityFF, robotRotationPID, robotTranslationPID, pathReplanConfig)


}

