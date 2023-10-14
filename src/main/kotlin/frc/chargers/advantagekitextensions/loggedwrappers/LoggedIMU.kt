package frc.chargers.advantagekitextensions.loggedwrappers

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.hardware.sensors.IMU
import frc.chargers.hardware.sensors.ThreeAxisAccelerometer
import frc.chargers.hardware.sensors.ThreeAxisSpeedometer
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope
import org.littletonrobotics.junction.Logger

/**
 * An IMU which has logging & replay capabilities from advantagekit.
 * 
 * Note: All of the Inputs classes also act as implementations of the 
 * [ThreeAxisGyroscope], [ThreeAxisAccelerometer] and [ThreeAxisSpeedometer] interfaces,
 * to reduce boilerplate code.
 */
public class LoggedIMU(
    private val imu: IMU
): IMU, SubsystemBase() {


    private val generalInputs = GeneralInputs()
    private inner class GeneralInputs: ChargerLoggableInputs() {
        var isConnected: Boolean by loggedBoolean(
            false,
            "isConnected"
        )
        var fusedHeading: Angle by loggedQuantity(
            0.0.degrees,
            "fusedHeadingDeg",
            degrees
        )
        var compassHeading: Angle by loggedQuantity(
            0.0.degrees,
            "compassHeadingDeg",
            degrees
        )
        var altitude: Distance? by loggedNullableQuantity(
            Distance(0.0),
            "altitudeMeters",
            meters
        )

        var name: String by loggedString(
            defaultValue = "",
            logName = "name"
        )

        fun update(){
            isConnected = imu.isConnected
            fusedHeading = imu.heading
            compassHeading = imu.compass.heading
            altitude = imu.altitude
            name = imu.name
        }
    }


    override val heading: Angle get() = generalInputs.fusedHeading
    override val isConnected: Boolean get() = generalInputs.isConnected
    override val altitude: Distance? get() = generalInputs.altitude
    override val name: String get() = generalInputs.name

    override val compass: HeadingProvider = object: HeadingProvider{
        override val heading: Angle get() = generalInputs.compassHeading
    }

    
    public inner class GyroscopeInputs: ChargerLoggableInputs(), ThreeAxisGyroscope{
        override var yaw: Angle by loggedQuantity(
            Angle(0.0),
            "yawDeg",
            degrees
        )
        override var pitch: Angle by loggedQuantity(
            Angle(0.0),
            "pitchDeg",
            degrees
        )
        override var roll: Angle by loggedQuantity(
            Angle(0.0),
            "rollDeg",
            degrees
        )
        override var heading: Angle by loggedQuantity(
            Angle(0.0),
            "continuousHeadingDeg",
            degrees
        )

        internal fun update(){
            val gyro = imu.gyroscope
            yaw = gyro.yaw
            pitch = gyro.pitch
            roll = gyro.roll
            heading = gyro.heading
        }
    }

    public inner class AccelerometerInputs: ChargerLoggableInputs(), ThreeAxisAccelerometer{
        override var xAcceleration: Acceleration by loggedQuantity(
            logName = "xAccel(MetersPerSecSquared)",
            logUnit = meters/seconds/seconds
        )

        override var yAcceleration: Acceleration by loggedQuantity(
            logName = "yAccel(MetersPerSecSquared)",
            logUnit = meters/seconds/seconds
        )

        override var zAcceleration: Acceleration by loggedQuantity(
            logName = "yAccel(MetersPerSecSquared)",
            logUnit = meters/seconds/seconds
        )
        
        internal fun update(){
            xAcceleration = imu.accelerometer.xAcceleration
            yAcceleration = imu.accelerometer.yAcceleration
            zAcceleration = imu.accelerometer.zAcceleration
        }
    }

    public inner class SpeedometerInputs: ChargerLoggableInputs(), ThreeAxisSpeedometer{
        override var xVelocity: Velocity by loggedQuantity(
            logName = "xVelocity(MetersPerSec)",
            logUnit = meters/seconds
        )

        override var yVelocity: Velocity by loggedQuantity(
            logName = "yVelocity(MetersPerSec)",
            logUnit = meters/seconds
        )

        override var zVelocity: Velocity by loggedQuantity(
            logName = "yVelocity(MetersPerSec)",
            logUnit = meters/seconds
        )

        internal fun update(){
            xVelocity = imu.speedometer.xVelocity
            yVelocity = imu.speedometer.yVelocity
            zVelocity = imu.speedometer.zVelocity
        }
    }
    
    


    override val gyroscope: GyroscopeInputs = GyroscopeInputs()
    override val accelerometer: AccelerometerInputs = AccelerometerInputs()
    override val speedometer: SpeedometerInputs = SpeedometerInputs()

    override fun reset() {
        imu.reset()
    }

    override fun periodic(){
        gyroscope.update()
        accelerometer.update()
        speedometer.update()
        generalInputs.update()

        // note: The gyroscope, speedometer and accelerometer all act as Inputs classes
        // in addition to holding public API values.
        Logger.getInstance().apply{
            processInputs(name,generalInputs)
            processInputs("$name/Gyroscope",gyroscope)
            processInputs("$name/Speedometer",speedometer)
            processInputs("$name/Accelerometer",accelerometer)
        }

    }




}