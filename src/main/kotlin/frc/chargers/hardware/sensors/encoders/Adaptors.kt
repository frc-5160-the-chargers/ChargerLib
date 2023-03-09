package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix.motorcontrol.IMotorController as CTREEncoder
import com.revrobotics.RelativeEncoder as RevEncoder
import edu.wpi.first.wpilibj.Encoder as WpilibEncoder

// This file contains a variety of adapters allowing various
// implementations of encoders from various different
// libraries and vendors (WPILib, REV, CTRE, etc.) to meet
// the ChargerLib Encoder interface.

public fun WpilibEncoder.asChargerEncoder(anglePerPulse: Angle): WPILibEncoderAdapter = WPILibEncoderAdapter(this, anglePerPulse)
public fun WpilibEncoder.asChargerEncoder(pulsesPerRotation: Int): WPILibEncoderAdapter = WPILibEncoderAdapter(this, pulsesPerRotation)
public fun RevEncoder.asChargerEncoder(): RevEncoderAdapter = RevEncoderAdapter(this)
public fun CTREEncoder.asChargerEncoder(pidIndex: Int, anglePerPulse: Angle): CTREMotorControllerEncoderAdapter = CTREMotorControllerEncoderAdapter(this, pidIndex, anglePerPulse)
public fun CTREEncoder.asChargerEncoder(pidIndex: Int, pulsesPerRotation: Int): CTREMotorControllerEncoderAdapter = CTREMotorControllerEncoderAdapter(this, pidIndex, pulsesPerRotation)

/**
 * An adapter from the WPILib Encoder class to the ChargerLib Encoder interface.
 */
public class WPILibEncoderAdapter(private val wpiLibEncoder: WpilibEncoder, private val anglePerPulse: Angle) : Encoder {
    public constructor(
        wpiLibEncoder: WpilibEncoder,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature*/
    ) : this(wpiLibEncoder, (1/pulsesPerRotation.toDouble()).ofUnit(Rotations))

    override val angularPosition: Angle
        get() = wpiLibEncoder.get() * anglePerPulse
    override val angularVelocity: AngularVelocity
        get() = wpiLibEncoder.rate * anglePerPulse / 1.seconds

    public val encoder: WpilibEncoder get() = wpiLibEncoder
}

/**
 * An adapter from the REV RelativeEncoder class to the ChargerLib Encoder interface.
 */
public class RevEncoderAdapter(private val revEncoder: RevEncoder) : Encoder, RevEncoder by revEncoder {
    override val angularPosition: Angle
        get() = revEncoder.position.ofUnit(Rotations)

    override val angularVelocity: AngularVelocity
        get() = revEncoder.velocity.ofUnit(Rotations/Minutes)
}

/**
 * An adapter from the CTRE Encoder class to the ChargerLib Encoder interface.
 */
public class CTREMotorControllerEncoderAdapter(
    private val ctreMotorController: CTREEncoder,
    private val pidIndex: Int,
    private val anglePerPulse: Angle
) : Encoder, CTREEncoder by ctreMotorController {
    public constructor(
        ctreMotorController: CTREEncoder,
        pidIndex: Int,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature */
    ) : this(ctreMotorController, pidIndex, (1/pulsesPerRotation.toDouble()).ofUnit(Rotations))

    override var angularPosition: Angle
        get() = ctreMotorController.getSelectedSensorPosition(pidIndex) * anglePerPulse
        set(value) {
            ctreMotorController.setSelectedSensorPosition((value/anglePerPulse).value, pidIndex, DEFAULT_TIMEOUT_MS) // TODO: Should throw if error code?
        }

    override val angularVelocity: AngularVelocity
        get() = ctreMotorController.getSelectedSensorVelocity(pidIndex) * anglePerPulse / timeBetweenPulses

    public companion object {
        private const val DEFAULT_TIMEOUT_MS = 500
        private val timeBetweenPulses = 100.milli.seconds
    }
}
