package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Velocity
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope
import frc.chargers.hardware.sensors.gyroscopes.ZeroableHeadingProvider
import frc.chargers.wpilibextensions.kinematics.xVelocity
import frc.chargers.wpilibextensions.kinematics.yVelocity

/**
 * Represents an [Inertial Measurement Unit](https://en.wikipedia.org/wiki/Inertial_measurement_unit),
 * with built-in gyroscope, accelerometer and speedometer functions.
 */
public interface IMU: ZeroableHeadingProvider {
    public fun reset()
    public val isConnected: Boolean
    public val altitude: Distance?
    public val imuName: String get() = "Generic IMU"

    public val gyroscope: ThreeAxisGyroscope
    public val compass: HeadingProvider
    public val accelerometer: ThreeAxisAccelerometer
    public val speedometer: ThreeAxisSpeedometer

}