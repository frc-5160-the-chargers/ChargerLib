package frc.chargers.hardware.configuration

/**
 * A class used to denote a piece of hardware(motor controllers, encoders, etc.)
 * that can be configured.
 */
public interface HardwareConfigurable<C: HardwareConfiguration> {
    public fun configure(configuration: C)
}