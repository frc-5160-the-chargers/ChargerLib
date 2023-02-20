package frc.chargers.hardware.sensors.gyroscopes

import com.batterystaple.kmeasure.quantities.Angle

public interface ThreeAxisGyroscope {
    public val yaw: Angle
    public val pitch: Angle
    public val roll: Angle
}