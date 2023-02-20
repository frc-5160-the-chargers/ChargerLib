package frc.chargers.controls

public interface Controller<out T> {
    public fun calculateOutput(): T
}