package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Acceleration

public interface ThreeAxisAccelerometer {
    public val xAcceleration: Acceleration
    public val yAcceleration: Acceleration
    public val zAcceleration: Acceleration
}