package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle

public fun PositionEncoder.withOffset(zeroOffset: Angle): PositionEncoder = object: PositionEncoder{
    override val angularPosition: Angle
        get() = this@withOffset.angularPosition - zeroOffset

}

public fun Encoder.withOffset(zeroOffset: Angle): Encoder = object: Encoder by this{
    override val angularPosition: Angle
        get() = this@withOffset.angularPosition - zeroOffset
}


