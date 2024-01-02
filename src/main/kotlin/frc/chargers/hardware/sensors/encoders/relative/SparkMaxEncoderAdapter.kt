package frc.chargers.hardware.sensors.encoders.relative

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.minutes
import com.batterystaple.kmeasure.units.rotations
import com.revrobotics.RelativeEncoder
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.revertIfInvalid

/**
 * An adapter from the REV RelativeEncoder class to the ChargerLib Encoder interface.
 */
public class SparkMaxEncoderAdapter(private val revEncoder: RelativeEncoder) : ResettableEncoder, RelativeEncoder by revEncoder {
    private var previousPosition = revEncoder.position.ofUnit(rotations)
    private var previousVelocity = AngularVelocity(0.0)

    override fun setZero(newZero: Angle){
        position = newZero.inUnit(rotations)
    }
    override val angularPosition: Angle
        get() = revEncoder.position.ofUnit(rotations)
            .revertIfInvalid(previousPosition)
            .also{ previousPosition = it }

    override val angularVelocity: AngularVelocity
        get() = revEncoder.velocity.ofUnit(rotations / minutes)
            .revertIfInvalid(previousVelocity)
            .also{ previousVelocity = it }
}