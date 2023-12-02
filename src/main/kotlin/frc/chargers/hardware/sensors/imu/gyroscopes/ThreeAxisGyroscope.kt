package frc.chargers.hardware.sensors.imu.gyroscopes

import com.batterystaple.kmeasure.quantities.Angle


/**
 * Represents a generic gyroscope, with measurements for yaw, pitch, and roll.
 */
public interface ThreeAxisGyroscope: HeadingProvider {
    public val yaw: Angle
    public val pitch: Angle
    public val roll: Angle

    override val heading: Angle
        get() = yaw

    /**
     * A [ThreeAxisGyroscope] that has static values for yaw, pitch and roll. These default to 0.
     */
    public class Static(
        override val yaw: Angle = Angle(0.0),
        override val pitch: Angle = Angle(0.0),
        override val roll: Angle = Angle(0.0)
    ): ThreeAxisGyroscope
}