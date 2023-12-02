package frc.chargers.hardware.configuration

/**
 * A marker interface used to denote a configuration for various kinds of hardware:
 * like encoders and motor controllers.
 */
public interface HardwareConfiguration

/**
 * A class used to denote a piece of hardware(motor controllers, encoders, etc.)
 * that can be configured.
 */
public interface HardwareConfigurable<C: HardwareConfiguration>{
    public fun configure(configuration: C)
}