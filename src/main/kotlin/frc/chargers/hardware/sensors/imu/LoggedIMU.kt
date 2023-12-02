package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.hardware.sensors.ThreeAxisAccelerometer
import frc.chargers.hardware.sensors.ThreeAxisSpeedometer
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import org.littletonrobotics.junction.Logger

/**
 * A wrapped [IMU] that provides sim and logging capabilities.
 */
public fun advantageKitIMU(
    name: String,
    realImpl: IMU,
    simImpl: IMU
): IMU = LoggedIMU(
    name,
    if (RobotBase.isSimulation()) simImpl else realImpl
)

/**
 * An IMU which has logging & replay capabilities from advantagekit.
 * 
 * Note: All the Inputs classes also act as implementations of the
 * [ThreeAxisGyroscope], [ThreeAxisAccelerometer] and [ThreeAxisSpeedometer] interfaces,
 * to reduce boilerplate code.
 */
private class LoggedIMU(
    private val imuName: String,
    private val imu: IMU
): IMU, SubsystemBase() {

    override val heading: Angle get() = generalInputs.fusedHeading
    override val isConnected: Boolean get() = generalInputs.isConnected
    override val altitude: Distance? get() = generalInputs.altitude
    override val compass: HeadingProvider = object: HeadingProvider {
        override val heading: Angle get() = generalInputs.compassHeading
    }
    override val gyroscope: GyroscopeInputs = GyroscopeInputs()
    override val accelerometer: AccelerometerInputs = AccelerometerInputs()
    override val speedometer: SpeedometerInputs = SpeedometerInputs()
    override fun zeroHeading(){ imu.zeroHeading() }
    override fun reset() { imu.reset() }









    private val generalInputs = GeneralInputs()
    private inner class GeneralInputs: ChargerLoggableInputs() {
        var isConnected: Boolean by loggedBoolean()
        var fusedHeading: Angle by loggedQuantity(
            logUnit = degrees,
            logName = "fusedHeadingDeg"
        )
        var compassHeading: Angle by loggedQuantity(
            logUnit = degrees,
            logName = "compassHeadingDeg"
        )
        var altitude: Distance? by loggedNullableQuantity(
            logUnit = meters,
            logName = "altitudeMeters"
        )
        var baseIMUName: String by loggedString()

        fun update(){
            isConnected = imu.isConnected
            fusedHeading = imu.heading
            compassHeading = imu.compass.heading
            altitude = imu.altitude
            baseIMUName = imuName
        }
    }

    
    inner class GyroscopeInputs: ChargerLoggableInputs(), ThreeAxisGyroscope {
        override var yaw: Angle by loggedQuantity(
            logUnit = degrees,
            logName = "yawDeg"
        )
        override var pitch: Angle by loggedQuantity(
            logUnit = degrees,
            logName = "pitchDeg"
        )
        override var roll: Angle by loggedQuantity(
            logUnit = degrees,
            logName = "rollDeg",
        )
        override var heading: Angle by loggedQuantity(
            logUnit = degrees,
            logName = "continuousHeadingDeg"
        )

        internal fun update(){
            val gyro = imu.gyroscope
            yaw = gyro.yaw
            pitch = gyro.pitch
            roll = gyro.roll
            heading = gyro.heading
        }
    }

    inner class AccelerometerInputs: ChargerLoggableInputs(), ThreeAxisAccelerometer {
        override var xAcceleration: Acceleration by loggedQuantity(
            logUnit = meters/seconds/seconds,
            logName = "xAccel(MetersPerSecSquared)"
        )

        override var yAcceleration: Acceleration by loggedQuantity(
            logUnit = meters/seconds/seconds,
            logName = "yAccel(MetersPerSecSquared)"
        )

        override var zAcceleration: Acceleration by loggedQuantity(
            logUnit = meters/seconds/seconds,
            logName = "zAccel(MetersPerSecSquared)"
        )
        
        internal fun update(){
            xAcceleration = imu.accelerometer.xAcceleration
            yAcceleration = imu.accelerometer.yAcceleration
            zAcceleration = imu.accelerometer.zAcceleration
        }
    }

    inner class SpeedometerInputs: ChargerLoggableInputs(), ThreeAxisSpeedometer {
        override var xVelocity: Velocity by loggedQuantity(
            logUnit = meters/seconds,
            logName = "xVelocity(MetersPerSec)"
        )

        override var yVelocity: Velocity by loggedQuantity(
            logUnit = meters/seconds,
            logName = "yVelocity(MetersPerSec)"
        )

        override var zVelocity: Velocity by loggedQuantity(
            logUnit = meters/seconds,
            logName = "zVelocity(MetersPerSec)"
        )

        internal fun update(){
            xVelocity = imu.speedometer.xVelocity
            yVelocity = imu.speedometer.yVelocity
            zVelocity = imu.speedometer.zVelocity
        }
    }


    override fun periodic(){
        gyroscope.update()
        accelerometer.update()
        speedometer.update()
        generalInputs.update()

        // note: The gyroscope, speedometer and accelerometer all act as Inputs classes
        // in addition to holding public API values.
        Logger.getInstance().apply{
            processInputs(imuName,generalInputs)
            processInputs("$imuName/Gyroscope",gyroscope)
            processInputs("$imuName/Speedometer",speedometer)
            processInputs("$imuName/Accelerometer",accelerometer)
        }
    }
}