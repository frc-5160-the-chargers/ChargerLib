package frc.chargers.hardware.motorcontrol

/**
 * A generic interface representing a piece of hardware and/or a motor
 * which can return its own temperature.
 */
public interface TemperatureProvider {
    public val tempCelsius: Double
}