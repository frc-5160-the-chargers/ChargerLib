package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Distance
import frc.chargers.hardware.sensors.ThreeAxisAccelerometer
import frc.chargers.hardware.sensors.ThreeAxisSpeedometer
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider

/**
 * Represents an [Inertial Measurement Unit](https://en.wikipedia.org/wiki/Inertial_measurement_unit),
 * with built-in gyroscope, accelerometer and speedometer functions.
 */
public interface IMU: ZeroableHeadingProvider {
    public fun reset()
    public val isConnected: Boolean
    public val altitude: Distance?

    public val gyroscope: ThreeAxisGyroscope
    public val compass: HeadingProvider
    public val accelerometer: ThreeAxisAccelerometer
    public val speedometer: ThreeAxisSpeedometer

}