package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Velocity

public interface ThreeAxisSpeedometer {
    public val xVelocity: Velocity
    public val yVelocity: Velocity
    public val zVelocity: Velocity
}