package frc.chargers.hardware.subsystemutils.differentialdrive

import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants

/**
 * A convenience class for holding control parameters of an [frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain].
 */
public open class DiffDriveControl(
    public val leftVelocityPID: PIDConstants,
    public val leftFF: AngularMotorFF,
    public val rightVelocityPID: PIDConstants,
    public val rightPID: AngularMotorFF,
    public val robotRotationPID: PIDConstants = PIDConstants(0.4,0.0,0.0),
    public val pathAlgorithm: PathAlgorithm = PathAlgorithm.LTV,
    public val pathReplanConfig: ReplanningConfig = ReplanningConfig(),
){
    public data object None: DiffDriveControl(
        PIDConstants(0.0,0.0,0.0),
        AngularMotorFF.None,
        PIDConstants(0.0,0.0,0.0),
        AngularMotorFF.None
    )

    public enum class PathAlgorithm{
        LTV, RAMSETE
    }

}
