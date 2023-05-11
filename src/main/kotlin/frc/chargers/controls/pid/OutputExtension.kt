package frc.chargers.controls.pid

public fun interface OutputExtension {
    public fun calculate(target: Double, error: Double): Double
}