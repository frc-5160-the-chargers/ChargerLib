package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Acceleration

/**
 * Represents a generic Accelerometer, with acceleration measurements for the x, y, and z axes.
 */
public interface ThreeAxisAccelerometer {
    public val xAcceleration: Acceleration
    public val yAcceleration: Acceleration
    public val zAcceleration: Acceleration


    /**
     * A [ThreeAxisAccelerometer] with static values for x, y, and z accelerations; these default to 0.
     */
    public class Static(
        override val xAcceleration: Acceleration = Acceleration(0.0),
        override val yAcceleration: Acceleration = Acceleration(0.0),
        override val zAcceleration: Acceleration = Acceleration(0.0)
    ): ThreeAxisAccelerometer
}