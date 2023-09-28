package frc.chargers.hardware.sensors.encoders

/**
 * A marker interface used to denote a configuration for an Encoder.
 */
public interface EncoderConfiguration

/**
 * A class used to denote an encoder
 * that can be configured.
 */
public interface EncoderConfigurable<C: EncoderConfiguration>{
    public fun configure(configuration: C)
}