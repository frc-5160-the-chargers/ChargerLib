package frc.chargers.hardware.sensors.encoders.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix.sensors.CANCoder
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.VelocityEncoder

public class ChargerCANCoder(id: Int, canbus: String = "") : CANCoder(id, canbus), Encoder {
    override var angularPosition: Angle
        get() = position.ofUnit(degrees)
        set(value) { position = value.inUnit(degrees) }

    override val angularVelocity: AngularVelocity
        get() = velocity.ofUnit(degrees/seconds)

    public val absolute: Encoder = AbsoluteEncoderAdapter()
    private inner class AbsoluteEncoderAdapter : Encoder, VelocityEncoder by this {
        override val angularPosition: Angle
            get() = absolutePosition.ofUnit(degrees)
    }
}