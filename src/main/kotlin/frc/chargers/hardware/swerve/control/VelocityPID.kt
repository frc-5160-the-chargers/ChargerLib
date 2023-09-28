package frc.chargers.hardware.swerve.control

import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants

/**
 * A class used to hold the necessary specifications
 * For velocity PID Control.
 */
public class VelocityPID(
    public val pidConstants: PIDConstants,
    public val ff: AngularMotorFF
)