package frc.chargers.hardware.sensors.cameras.vision

/**
 * Represents a generic system that can report a classified result of any kind.
 */
public interface Classifier<T> {
    /**
     * The result obtained from the classifier.
     */
    public val itemType: T

    /**
     * A get-set variable used to control whether the classifier's overarching
     * vision camera is required or not.
     */
    public var isRequired: Boolean

    /**
     * Resets the pipeline of the overarching vision camera to the appropriate value
     * for this pipeline.
     */
    public fun reset()
}