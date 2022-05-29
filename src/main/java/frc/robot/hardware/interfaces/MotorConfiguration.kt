package frc.robot.hardware.interfaces

/**
 * A marker interface for classes that store configuration
 * parameters for motors.
 */
public interface MotorConfiguration

/**
 * Indicates a class with motors that can be configured.
 *
 * Examples include motor controllers, motor controller groups,
 * and various subsystems.
 */
public interface MotorConfigurable<C : MotorConfiguration> {
    public fun configure(configuration: C)
}