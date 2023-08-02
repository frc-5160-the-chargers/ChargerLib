package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile


/**
 * An encoder motor controller with builtin PID support.
 */
public interface FeedbackMotorController: EncoderMotorController{
    public fun setAngularVelocity(velocity: AngularVelocity, pidConstants: PIDConstants, feedforward: AngularMotorFF)
    public fun setAngularPosition(position: Angle, pidConstants: PIDConstants)
    public fun setAngularPosition(position: Angle, pidConstants: PIDConstants, feedforward: AngularMotorFF, constraints: AngularTrapezoidProfile.Constraints)
}