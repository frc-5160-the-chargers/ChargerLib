package frc.chargers.hardware.sensors.encoders

public interface ZeroableEncoder : Encoder {
    /**
     * Sets the current angular position to be the new zero.
     */
    public fun setZero()
}