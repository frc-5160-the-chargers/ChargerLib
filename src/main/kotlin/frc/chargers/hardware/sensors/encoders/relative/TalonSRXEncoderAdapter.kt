package frc.chargers.hardware.sensors.encoders.relative

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix.motorcontrol.IMotorController
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * An adapter from the phoenix v5 encoder class to the ChargerLib Encoder interface.
 */
public class TalonSRXEncoderAdaptor(
    private val ctreMotorController: IMotorController,
    private val pidIndex: Int,
    private val anglePerPulse: Angle
) : Encoder, IMotorController by ctreMotorController {
    public constructor(
        ctreMotorController: IMotorController,
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