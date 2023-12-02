package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.ThreeAxisAccelerometer
import frc.chargers.hardware.sensors.ThreeAxisSpeedometer
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import frc.chargers.wpilibextensions.kinematics.xVelocity
import frc.chargers.wpilibextensions.kinematics.yVelocity

/**
 * Represents a Sim IMU, with dummy values for most outputs.
 *
 * Contains headingProviderImpl and getChassisSpeeds in order to use simulated readings from the drivetrain
 * as a stand-in for values.
 */
public class IMUSim: IMU {

    public companion object{
        private var headingProviderImpl: HeadingProvider = HeadingProvider { Angle(0.0) }
        private var getChassisSpeeds: () -> ChassisSpeeds = { ChassisSpeeds(0.0,0.0,0.0) }
        public fun setHeadingSource(source: HeadingProvider){
            headingProviderImpl = source
        }
        public fun setChassisSpeedsSource(source: () -> ChassisSpeeds){
            getChassisSpeeds = source
        }
    }


    private var previousChassisSpeeds = ChassisSpeeds()
    private var angleOffset = Angle(0.0)
    override fun reset() {}

    override fun zeroHeading(){
        angleOffset = headingProviderImpl.heading
    }

    override val isConnected: Boolean = false
    override val altitude: Distance? = null
    override val gyroscope: ThreeAxisGyroscope = object: ThreeAxisGyroscope {
        override val yaw: Angle get() = -(headingProviderImpl.heading - angleOffset)
        override val pitch: Angle = Angle(0.0)
        override val roll: Angle = Angle(0.0)

    }
    override val compass: HeadingProvider = object: HeadingProvider {
        override val heading: Angle get() = Angle(0.0)
    }
    override val accelerometer: ThreeAxisAccelerometer = object: ThreeAxisAccelerometer {
        override val xAcceleration: Acceleration
            get(){
                val speeds = getChassisSpeeds()
                val speedDelta = speeds.xVelocity - previousChassisSpeeds.xVelocity
                previousChassisSpeeds = speeds
                return speedDelta / ChargerRobot.LOOP_PERIOD
            }
        override val yAcceleration: Acceleration
            get(){
                val speeds = getChassisSpeeds()
                val speedDelta = speeds.yVelocity - previousChassisSpeeds.yVelocity
                previousChassisSpeeds = speeds
                return speedDelta / ChargerRobot.LOOP_PERIOD
            }

        override val zAcceleration: Acceleration = Acceleration(0.0)
    }
    override val speedometer: ThreeAxisSpeedometer = object: ThreeAxisSpeedometer {
        override val xVelocity: Velocity get() = getChassisSpeeds().xVelocity
        override val yVelocity: Velocity get() = getChassisSpeeds().yVelocity
        override val zVelocity: Velocity = Velocity(0.0)
    }
    override val heading: Angle get() = headingProviderImpl.heading - angleOffset

}