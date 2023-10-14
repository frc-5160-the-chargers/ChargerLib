package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Distance
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope

/**
 * Represents an [Inertial Measurement Unit](https://en.wikipedia.org/wiki/Inertial_measurement_unit),
 * with built-in gyroscope, accelerometer and speedometer functions.
 */
public interface IMU: HeadingProvider {
    public fun reset()
    public val isConnected: Boolean
    public val altitude: Distance?
    public val name: String get() = "Generic IMU"

    public val gyroscope: ThreeAxisGyroscope
    public val compass: HeadingProvider
    public val accelerometer: ThreeAxisAccelerometer
    public val speedometer: ThreeAxisSpeedometer

}