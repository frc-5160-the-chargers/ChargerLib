package frc.chargers.hardware.sensors.encoders.relative

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.hardware.TalonFX
import frc.chargers.hardware.sensors.encoders.ResettableTimestampedEncoder
import frc.chargers.utils.Measurement

/**
 * Adapts the TalonFX motor class's builtin encoder functionality to the charger encoder interface.
 * Also used for fusedCANCoder applications
 *
 */
public class TalonFXEncoderAdapter(
    private val motorController: TalonFX
): ResettableTimestampedEncoder {

    override fun setZero(newZero: Angle) {
        motorController.setPosition(newZero.inUnit(rotations))
    }


    override val timestampedAngularPosition: Measurement<Angle>
        get(){
            val statusSignal = motorController.rotorPosition
            return Measurement(
                value = statusSignal.value.ofUnit(rotations),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds)
            )
        }
    override val timestampedAngularVelocity: Measurement<AngularVelocity>
        get(){
            val statusSignal = motorController.rotorVelocity
            return Measurement(
                value = statusSignal.value.ofUnit(rotations / seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds)
            )
        }

}