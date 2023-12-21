package frc.chargers.hardware.subsystemutils.differentialdrive

import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants

/**
 * A convenience class for holding control parameters of an [frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain].
 */
public data class DiffDriveControl(
    val leftVelocityPID: PIDConstants,
    val leftFF: AngularMotorFF,
    val rightVelocityPID: PIDConstants,
    val rightPID: AngularMotorFF,
    val robotRotationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    val robotTranslationPID: PIDConstants = PIDConstants(0.3,0.0,0.0),
    val pathReplanConfig: ReplanningConfig = ReplanningConfig()
){
    public companion object{
        public val None: DiffDriveControl = DiffDriveControl(
            PIDConstants(0.0,0.0,0.0),
            AngularMotorFF.None,
            PIDConstants(0.0,0.0,0.0),
            AngularMotorFF.None
        )
    }

}
