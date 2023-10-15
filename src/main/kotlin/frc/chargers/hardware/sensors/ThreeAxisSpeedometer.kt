package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Velocity

public interface ThreeAxisSpeedometer {
    public val xVelocity: Velocity
    public val yVelocity: Velocity
    public val zVelocity: Velocity

    public class None: ThreeAxisSpeedometer{
        override val xVelocity: Velocity = Velocity(0.0)
        override val yVelocity: Velocity = Velocity(0.0)
        override val zVelocity: Velocity = Velocity(0.0)
    }
}