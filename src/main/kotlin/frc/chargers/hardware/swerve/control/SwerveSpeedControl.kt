package frc.chargers.hardware.swerve.control

import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants

/**
 * A class used to hold the necessary specifications
 * For velocity PID Control in a [frc.chargers.hardware.swerve.module.SwerveModule].
 */
public class SwerveSpeedControl(
    public val pidConstants: PIDConstants,
    public val ff: AngularMotorFF
)