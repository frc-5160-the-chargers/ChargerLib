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
        var isConnected: Boolean by loggedBoolean(logName = "isConnected")
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
            logName = "altitudeMeters",
            defaultValue = Distance(0.0)
        )
        var baseIMUName: String by loggedString(logName = "name")

        fun update(){
            isConnected = imu.isConnected
            fusedHeading = imu.heading
            compassHeading = imu.compass.heading
            altitude = imu.altitude
            baseIMUName = imu.imuName
        }
    }


    override val heading: Angle get() = generalInputs.fusedHeading
    override val isConnected: Boolean get() = generalInputs.isConnected
    override val altitude: Distance? get() = generalInputs.altitude


    // getter cannot be used here; will cause accidental override kotlin error
    // with the getName() function of SubsystemBase()
    override val imuName: String
        get() = generalInputs.baseIMUName

    override val compass: HeadingProvider = object: HeadingProvider{
        override val heading: Angle get() = generalInputs.compassHeading
    }

    
    public inner class GyroscopeInputs: ChargerLoggableInputs(), ThreeAxisGyroscope{
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

    public inner class AccelerometerInputs: ChargerLoggableInputs(), ThreeAxisAccelerometer{
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

    public inner class SpeedometerInputs: ChargerLoggableInputs(), ThreeAxisSpeedometer{
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
            processInputs(imuName,generalInputs)
            processInputs("$imuName/Gyroscope",gyroscope)
            processInputs("$imuName/Speedometer",speedometer)
            processInputs("$imuName/Accelerometer",accelerometer)
        }

    }




}