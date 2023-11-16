package frc.chargers.controls

/**
 * A [Controller] that uses feedback control to calculate its output.
 */
public interface FeedbackController<I, out T>: Controller<T>{
    public var target: I

    public val error: I
}