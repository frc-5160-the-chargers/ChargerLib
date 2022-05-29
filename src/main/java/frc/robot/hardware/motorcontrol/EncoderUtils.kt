package frc.robot.hardware.motorcontrol

import com.batterystaple.kmeasure.Angle
import com.batterystaple.kmeasure.AngularVelocity
import com.batterystaple.kmeasure.average
import frc.robot.hardware.interfaces.Encoder
import frc.robot.hardware.interfaces.EncoderMotorController

/**
 * An abstraction of multiple encoders into one.
 *
 * The output of its position and velocity are the average
 * of the positions and velocities of its composite encoders.
 */
public class AverageEncoder(private vararg val encoders: Encoder) : Encoder {
    public constructor(vararg motorControllers: EncoderMotorController) : this(*motorControllers.map(EncoderMotorController::encoder).toTypedArray())

    override val angularPosition: Angle
        get() = encoders.map(Encoder::angularPosition).average()
    override val angularVelocity: AngularVelocity
        get() = encoders.map(Encoder::angularVelocity).average()
}