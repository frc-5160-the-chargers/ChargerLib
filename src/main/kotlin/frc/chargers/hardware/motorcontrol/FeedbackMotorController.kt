package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints


/**
 * An encoder motor controller with builtin PID support.
 */
public interface FeedbackMotorController: EncoderMotorController{
    /**
     * Unlike position control, velocity control usually does not use external absolute encoders.
     * So, an absoluteEncoder variable here is not necessary.
     */
    public fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF
    )

    /**
     * If [absoluteEncoder] is null,
     * then the FeedbackMotorController will default to using the built-in encoder
     * for position measuring.
     */
    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        absoluteEncoder: PositionEncoder?
    )

    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants
    ): Unit = setAngularPosition(
        target,
        pidConstants,
        null
    )

    /**
     * If [absoluteEncoder] is null,
     * then the FeedbackMotorController will default to using the built-in encoder
     * for position measuring.
     */
    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF,
        constraints: AngularMotionConstraints,
        absoluteEncoder: PositionEncoder?
    )

    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        feedforward: AngularMotorFF,
        constraints: AngularMotionConstraints
    ): Unit = setAngularPosition(
        target,
        pidConstants,
        feedforward,
        constraints,
        null
    )


}