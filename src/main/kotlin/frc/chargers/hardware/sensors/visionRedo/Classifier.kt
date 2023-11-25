package frc.chargers.hardware.sensors.visionRedo

/**
 * Represents a generic system that can report a classified result of any kind.
 */
public interface Classifier<T> {
    public val itemType: T
}