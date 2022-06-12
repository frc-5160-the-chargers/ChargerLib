package frc.robot.utils.pid

public fun interface FeedForward {
    public fun calculate(target: Double, error: Double): Double
}