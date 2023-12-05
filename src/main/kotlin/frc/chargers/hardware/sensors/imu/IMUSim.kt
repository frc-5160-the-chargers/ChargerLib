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

public class IMUSim: IMU {

    /**
     * The specified LogTabs below are in fact LoggableInputsProvider's,
     * which automatically handle AdvantageKit logging and replay support(and work essentially as getters).
     * @see [frc.chargers.advantagekitextensions.LoggableInputsProvider]
     */

    private var previousXVelocity = Velocity(0.0)
    private var previousYVelocity = Velocity(0.0)
    private var angleOffset = Angle(0.0)

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

    override fun reset() {}
    override fun zeroHeading(){ angleOffset = headingProviderImpl.heading }


    override val isConnected: Boolean by IMU_INPUTS.boolean{false}
    override val altitude: Distance? by IMU_INPUTS.nullableQuantity{null}
    override val heading: Angle by IMU_INPUTS.quantity{headingProviderImpl.heading - angleOffset}

    override val gyroscope: ThreeAxisGyroscope = object: ThreeAxisGyroscope {
        override val yaw: Angle by GYRO_INPUTS.quantity{ -(headingProviderImpl.heading - angleOffset) }
        override val pitch: Angle by GYRO_INPUTS.quantity{ Angle(0.0) }
        override val roll: Angle by GYRO_INPUTS.quantity{ Angle(0.0) }
    }

    override val compass: HeadingProvider = object: HeadingProvider {
        override val heading: Angle by COMPASS_INPUTS.quantity{ Angle(0.0) }
    }

    override val accelerometer: ThreeAxisAccelerometer = object: ThreeAxisAccelerometer {
        override val xAcceleration: Acceleration by ACCELEROMETER_INPUTS.quantity{
            val speeds: ChassisSpeeds = getChassisSpeeds()
            val speedDelta = abs(speeds.xVelocity) - abs(previousXVelocity)
            previousXVelocity = speeds.xVelocity
            // return value
            speedDelta / ChargerRobot.LOOP_PERIOD
        }
        override val yAcceleration: Acceleration by ACCELEROMETER_INPUTS.quantity{
            val speeds = getChassisSpeeds()
            val speedDelta = abs(speeds.yVelocity) - abs(previousYVelocity)
            previousYVelocity = speeds.yVelocity
            // return value
            speedDelta / ChargerRobot.LOOP_PERIOD
        }
        override val zAcceleration: Acceleration by ACCELEROMETER_INPUTS.quantity{Acceleration(0.0)}
    }

    override val speedometer: ThreeAxisSpeedometer = object: ThreeAxisSpeedometer {
        override val xVelocity: Velocity by SPEEDOMETER_INPUTS.quantity{getChassisSpeeds().xVelocity}
        override val yVelocity: Velocity by SPEEDOMETER_INPUTS.quantity{getChassisSpeeds().yVelocity}
        override val zVelocity: Velocity by SPEEDOMETER_INPUTS.quantity{Velocity(0.0)}
    }

}