package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees

/**
 * An encoder that allows resetting zero values.
 * Particularly useful for absolute encoders or for relative encoders after homing.
 *
 * Depending on the underlying implementation, new zeroes
 * may or may not persist between restarts or runs of the program.
 */
public interface ResettableEncoder : ZeroableEncoder {
    /**
     * Sets the specified angular position to be the new zero.
     */
    public fun setZero(newZero: Angle)

    override fun setZero() { setZero(angularPosition) }
}

/**
 * an encoder which can both be reset and which provides timestamps.
 *
 * @see TimestampedEncoder
 * @see ResettableEncoder
 */
public interface ResettableTimestampedEncoder: ResettableEncoder, TimestampedEncoder

public class ZeroableEncoderOffsetManager(private val zeroableEncoder: ZeroableEncoder): ResettableEncoder, ZeroableEncoder by zeroableEncoder {
    public var offset: Angle = 0.degrees
    public override fun setZero() {
        offset = 0.degrees
    }

    public override fun setZero(newZero: Angle) {
        offset -= newZero // Seems kinda weird, but I did the math and I'm pretty sure this is the correct formula
    }
}