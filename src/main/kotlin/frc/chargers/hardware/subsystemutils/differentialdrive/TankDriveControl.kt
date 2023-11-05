package frc.chargers.hardware.subsystemutils.differentialdrive

import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants

public data class TankDriveControl(
    val leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    val leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    val rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    val rightMotorFF: AngularMotorFF = AngularMotorFF.None,
)