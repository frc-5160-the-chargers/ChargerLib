package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider

public fun PositionEncoder.withOffset(zeroOffset: Angle): PositionEncoder = object: PositionEncoder {
    override val angularPosition: Angle
        get() = this@withOffset.angularPosition - zeroOffset

}

public fun Encoder.withOffset(zeroOffset: Angle): Encoder = object: Encoder by this{
    override val angularPosition: Angle
        get() = this@withOffset.angularPosition - zeroOffset
}

public fun HeadingProvider.withOffset(zeroOffset: Angle): HeadingProvider = HeadingProvider{heading - zeroOffset}


