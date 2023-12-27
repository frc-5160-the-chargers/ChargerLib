package frc.chargers.hardware.sensors.encoders.relative

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.hardware.TalonFX
import frc.chargers.hardware.sensors.encoders.ResettableTimestampedEncoder
import frc.chargers.utils.QuantityMeasurement

/**
 * Adapts the TalonFX motor class's builtin encoder functionality to the charger encoder interface.
 * Also used for fusedCANCoder applications
 */
public class TalonFXEncoderAdapter(
    private val motorController: TalonFX
): ResettableTimestampedEncoder {

    override fun setZero(newZero: Angle) {
        motorController.setPosition(newZero.inUnit(rotations))
    }

    private val positionSignal = motorController.position
    private val velocitySignal = motorController.velocity

    override val timestampedAngularPosition: QuantityMeasurement<AngleDimension>
        get(){
            val statusSignal = positionSignal.refresh(true)
            return QuantityMeasurement(
                value = statusSignal.value.ofUnit(rotations),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds)
            )
        }

    override val timestampedAngularVelocity: QuantityMeasurement<AngularVelocityDimension>
        get(){
            val statusSignal = velocitySignal.refresh()
            return QuantityMeasurement(
                value = statusSignal.value.ofUnit(rotations / seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds)
            )
        }

}