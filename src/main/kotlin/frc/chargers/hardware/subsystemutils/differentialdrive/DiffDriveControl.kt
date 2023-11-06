package frc.chargers.hardware.subsystemutils.differentialdrive

import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants

/**
 * A convenience class for holding control parameters of an [frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain].
 */
public data class DiffDriveControl(
    val leftVelocityConstants: PIDConstants,
    val leftMotorFF: AngularMotorFF,
    val rightVelocityConstants: PIDConstants,
    val rightMotorFF: AngularMotorFF,
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
