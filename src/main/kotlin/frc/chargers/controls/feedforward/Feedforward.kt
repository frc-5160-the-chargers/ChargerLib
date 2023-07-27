package frc.chargers.controls.feedforward

/**
 * Represents a Generic Feedforward.
 * A feedforward is an equation which estimates the Control Effort(E) required
 * to achieve a certain output(O)
 */
public interface Feedforward<O,E> {
    public fun calculate(value: O): E
}