package frc.chargers.controls

public interface FeedbackController<I, out T>: Controller<T>{
    public var target: I

    public val error: I
}