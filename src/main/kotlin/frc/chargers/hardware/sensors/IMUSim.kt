package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Velocity
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope
import frc.chargers.wpilibextensions.kinematics.xVelocity
import frc.chargers.wpilibextensions.kinematics.yVelocity

/**
 * Represents a Sim IMU, with dummy values for most outputs.
 *
 * Contains headingProviderImpl and getChassisSpeeds in order to use simulated readings from the drivetrain
 * as a stand-in for values.
 */
public class IMUSim(
    private val headingProviderImpl: HeadingProvider,
    private val getChassisSpeeds: () -> ChassisSpeeds = { ChassisSpeeds(0.0,0.0,0.0) }
): IMU{
    override fun reset() {

    }

    override val imuName: String = "Sim IMU"
    override val isConnected: Boolean = false
    override val altitude: Distance? = null
    override val gyroscope: ThreeAxisGyroscope = object: ThreeAxisGyroscope {
        override val yaw: Angle get() = headingProviderImpl.heading
        override val pitch: Angle = Angle(0.0)
        override val roll: Angle = Angle(0.0)

    }
    override val compass: HeadingProvider = object: HeadingProvider {
        override val heading: Angle get() = Angle(0.0)
    }
    override val accelerometer: ThreeAxisAccelerometer = object: ThreeAxisAccelerometer{
        override val xAcceleration: Acceleration = Acceleration(0.0)
        override val yAcceleration: Acceleration = Acceleration(0.0)
        override val zAcceleration: Acceleration = Acceleration(0.0)

    }
    override val speedometer: ThreeAxisSpeedometer = object: ThreeAxisSpeedometer{
        override val xVelocity: Velocity
            get() = getChassisSpeeds().xVelocity
        override val yVelocity: Velocity
            get() = getChassisSpeeds().yVelocity
        override val zVelocity: Velocity = Velocity(0.0)
    }
    override val heading: Angle get() = headingProviderImpl.heading

}