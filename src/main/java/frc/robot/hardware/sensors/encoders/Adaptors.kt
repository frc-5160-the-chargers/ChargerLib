package frc.robot.hardware.sensors.encoders

import com.batterystaple.kmeasure.*
import com.ctre.phoenix.motorcontrol.IMotorController
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj.Encoder as WpilibEncoder

// This file contains a variety of adapters allowing various
// implementations of encoders from various different
// libraries and vendors (WPILib, REV, CTRE, etc.) to meet
// the ChargerLib Encoder interface.

/**
 * An adapter from the WPILib Encoder class to the ChargerLib Encoder interface.
 */
public class WPILibEncoderAdapter(private val wpiLibEncoder: WpilibEncoder, private val anglePerPulse: Angle) :
    Encoder {
    public constructor(
        wpiLibEncoder: WpilibEncoder,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature*/
    ) : this(wpiLibEncoder, (1/pulsesPerRotation.toDouble()).ofUnit(Rotations))

    override val angularPosition: Angle
        get() = wpiLibEncoder.get() * anglePerPulse
    override val angularVelocity: AngularVelocity
        get() = wpiLibEncoder.rate * anglePerPulse / 1.seconds

}

/**
 * An adapter from the REV RelativeEncoder class to the ChargerLib Encoder interface.
 */
public class RevEncoderAdapter(private val revEncoder: RelativeEncoder) : Encoder {
    override val angularPosition: Angle
        get() = revEncoder.position.ofUnit(Rotations)
    override val angularVelocity: AngularVelocity
        get() = revEncoder.velocity.ofUnit(Rotations/Minutes)
}

/**
 * An adapter from the CTRE Encoder class to the ChargerLib Encoder interface.
 */
public class CTREMotorControllerEncoderAdapter(private val ctreMotorController: IMotorController, private val pidIndex: Int, private val anglePerPulse: Angle) :
    Encoder {
    public constructor(
        ctreMotorController: IMotorController,
        pidIndex: Int,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature*/
    ) : this(ctreMotorController, pidIndex, (1/pulsesPerRotation.toDouble()).ofUnit(Rotations))

    override val angularPosition: Angle
        get() = ctreMotorController.getSelectedSensorPosition(pidIndex) * anglePerPulse

    override val angularVelocity: AngularVelocity
        get() = ctreMotorController.getSelectedSensorVelocity(pidIndex) * anglePerPulse / ctreTimeBetweenPulses
}

private val ctreTimeBetweenPulses = 100.milli.seconds