package frc.chargers.controls

/**
 * Represents a generic controller, which can produce a certain output when called.
 */
public interface Controller<out T> {
    public fun calculateOutput(): T
}