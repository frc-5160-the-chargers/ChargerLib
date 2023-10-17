package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Velocity

/**
 * Represents a Speedometer that can measure velocity in the x, y and z directions.
 */
public interface ThreeAxisSpeedometer {
    public val xVelocity: Velocity
    public val yVelocity: Velocity
    public val zVelocity: Velocity

    /**
     * A [ThreeAxisSpeedometer] with static values for x, y, and z velocity; these default to 0.
     */
    public class Static(
        override val xVelocity: Velocity = Velocity(0.0),
        override val yVelocity: Velocity = Velocity(0.0),
        override val zVelocity: Velocity = Velocity(0.0)
    ): ThreeAxisSpeedometer
}