package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.Utils

import com.ctre.phoenix6.hardware.TalonFX
import frc.chargers.utils.Measurement

import com.ctre.phoenix6.hardware.CANcoder as CTRECANcoder
import com.revrobotics.RelativeEncoder as RevEncoder
import edu.wpi.first.wpilibj.Encoder as WpilibEncoder

import com.ctre.phoenix.motorcontrol.IMotorController as TalonSRXEncoder

// This file contains a variety of adapters allowing various
// implementations of encoders from various different
// libraries and vendors (WPILib, REV, CTRE, etc.) to meet
// the ChargerLib Encoder interface.

// Note: The CTREMotorControllerEncoderAdaptor is not included in the functions below,
// as it is always an encoder which is bundled in a ChargerCANTalonFX class.

public fun WpilibEncoder.asChargerEncoder(anglePerPulse: Angle): WPILibEncoderAdapter = WPILibEncoderAdapter(this, anglePerPulse)
public fun WpilibEncoder.asChargerEncoder(pulsesPerRotation: Int): WPILibEncoderAdapter = WPILibEncoderAdapter(this, pulsesPerRotation)
public fun RevEncoder.asChargerEncoder(): RevEncoderAdapter = RevEncoderAdapter(this)
public fun CTRECANcoder.asChargerEncoder(): CTREEncoderAdapter = CTREEncoderAdapter(this.deviceID,this.canBus)


/**
 * An adapter from the WPILib Encoder class to the ChargerLib Encoder interface.
 */
public class WPILibEncoderAdapter(private val wpiLibEncoder: WpilibEncoder, private val anglePerPulse: Angle) : Encoder {
    public constructor(
        wpiLibEncoder: WpilibEncoder,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature*/
    ) : this(wpiLibEncoder, (1/pulsesPerRotation.toDouble()).ofUnit(rotations))

    override val angularPosition: Angle
        get() = wpiLibEncoder.get() * anglePerPulse
    override val angularVelocity: AngularVelocity
        get() = wpiLibEncoder.rate * anglePerPulse / 1.seconds

    public val encoder: WpilibEncoder get() = wpiLibEncoder
}

/**
 * An adapter from the REV RelativeEncoder class to the ChargerLib Encoder interface.
 */
public class RevEncoderAdapter(private val revEncoder: RevEncoder) : ResettableEncoder, RevEncoder by revEncoder {

    override fun setZero(newZero: Angle){
        position = newZero.inUnit(rotations)
    }
    override val angularPosition: Angle
        get() = revEncoder.position.ofUnit(rotations)

    override val angularVelocity: AngularVelocity
        get() = revEncoder.velocity.ofUnit(rotations/minutes)

}




/**
 * Adapts CTRE's solo CAN coder class to the Charger Encoder interface.
 */
public class CTREEncoderAdapter(
    deviceID: Int,
    canBus: String? = null
): CTRECANcoder(deviceID, canBus), TimestampedEncoder, ResettableEncoder{

    override fun setZero(newZero: Angle){
        setPosition(newZero.inUnit(rotations))
    }
    override val timestampedAngularPosition: Measurement<Angle>
        get(){
            val statusSignal = absolutePosition
            return Measurement(
                value = statusSignal.value.ofUnit(rotations),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = absolutePosition.timestamp.isValid
            )
        }

    override val timestampedAngularVelocity: Measurement<AngularVelocity>
        get(){
            val statusSignal = velocity
            return Measurement(
                value = statusSignal.value.ofUnit(rotations/seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = absolutePosition.timestamp.isValid
            )
        }
    override val angularPosition: Angle
        get() = absolutePosition.value.ofUnit(rotations)
    override val angularVelocity: AngularVelocity
        get() = velocity.value.ofUnit(rotations/seconds)

}

/**
 * Adapts the TalonFX motor class's builtin encoder functionality to the charger encoder interface.
 * Also used for fusedCANCoder applications
 *
 */
public class TalonFXEncoderAdapter(
    private val motorController: TalonFX
): TimestampedEncoder{

    override val angularPosition: Angle
        get() = motorController.rotorPosition.value.ofUnit(rotations)

    override val angularVelocity: AngularVelocity
        get() = motorController.rotorVelocity.value.ofUnit(rotations/seconds)

    override val timestampedAngularPosition: Measurement<Angle>
        get(){
            val statusSignal = motorController.rotorPosition
            return Measurement(
                value = statusSignal.value.ofUnit(rotations),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = statusSignal.timestamp.isValid
            )
        }
    override val timestampedAngularVelocity: Measurement<AngularVelocity>
        get(){
            val statusSignal = motorController.rotorVelocity
            return Measurement(
                value = statusSignal.value.ofUnit(rotations/seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = statusSignal.timestamp.isValid
            )
        }

}

/**
 * An adapter from the phoenix v5 encoder class to the ChargerLib Encoder interface.
 */
public class TalonSRXEncoderAdaptor(
    private val ctreMotorController: TalonSRXEncoder,
    private val pidIndex: Int,
    private val anglePerPulse: Angle
) : Encoder, TalonSRXEncoder by ctreMotorController {
    public constructor(
        ctreMotorController: TalonSRXEncoder,
        pidIndex: Int,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature */
    ) : this(ctreMotorController, pidIndex, (1/pulsesPerRotation.toDouble()).ofUnit(rotations))

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




