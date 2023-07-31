package frc.chargers.controls.feedforward

/**
 * Represents a Generic Feedforward.
 * A feedforward is an equation which estimates the Control Effort(type E) required
 * to achieve a certain output(type O).
 *
 *
 * See [here](https://www.controleng.com/articles/feed-forwards-augment-pid-control/) for an explanation of feedforward.
 */
public fun interface Feedforward<O,E> {
    public fun calculate(value: O): E
}