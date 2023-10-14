package frc.chargers.hardware.sensors.gyroscopes

import com.batterystaple.kmeasure.quantities.Angle

public interface ThreeAxisGyroscope: HeadingProvider {
    public val yaw: Angle
    public val pitch: Angle
    public val roll: Angle

    override val heading: Angle
        get() = yaw
}