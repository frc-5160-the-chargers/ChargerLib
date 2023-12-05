package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.kauailabs.navx.frc.AHRS
import frc.chargers.hardware.sensors.ThreeAxisAccelerometer
import frc.chargers.hardware.sensors.ThreeAxisSpeedometer
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import frc.chargers.utils.math.units.g

public class NavX(public val ahrs: AHRS = AHRS()) : IMU {

    /**
     * The specified LogTabs below are in fact LoggableInputsProvider's,
     * which automatically handle AdvantageKit logging and replay support(and work essentially as getters).
     * @see [frc.chargers.advantagekitextensions.LoggableInputsProvider]
     */

    /**
     * Creates a new NavX.
     */
    init {
        reset()
    }

    override fun zeroHeading(){
        ahrs.zeroYaw()
    }

    override fun reset() {
        ahrs.reset()
    }

    override val heading: Angle by IMU_INPUTS.quantity{
        -ahrs.fusedHeading.toDouble().ofUnit(degrees)
    } // Negative sign because the navX reports clockwise as positive, whereas we want counterclockwise to be positive

    override val altitude: Distance? by IMU_INPUTS.nullableQuantity{
        if (ahrs.isAltitudeValid) ahrs.altitude.toDouble().ofUnit(meters) else null
    }

    public val firmwareVersion: String get() = ahrs.firmwareVersion

    override val isConnected: Boolean by IMU_INPUTS.boolean{ ahrs.isConnected }

    override val gyroscope: Gyroscope = Gyroscope()
    override val compass: Compass = Compass()
    override val accelerometer: Accelerometer = Accelerometer()
    override val speedometer: Speedometer = Speedometer()

    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope {
        override val yaw: Angle by GYRO_INPUTS.quantity{
            ahrs.yaw.toDouble().ofUnit(degrees)
        }
        override val pitch: Angle by GYRO_INPUTS.quantity{
            ahrs.pitch.toDouble().ofUnit(degrees)
        }
        override val roll: Angle by GYRO_INPUTS.quantity{
            ahrs.roll.toDouble().ofUnit(degrees)
        }
        override val heading: Angle by GYRO_INPUTS.quantity{
            ahrs.angle.ofUnit(degrees)
        }
    }

    public inner class Compass internal constructor(): HeadingProvider {
        public override val heading: Angle by COMPASS_INPUTS.quantity{
            -ahrs.compassHeading.toDouble().ofUnit(degrees)
        } // Negative sign because the navX reports clockwise as positive
        // whereas we want counterclockwise to be positive
    }

    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer {
        override val xAcceleration: Acceleration by ACCELEROMETER_INPUTS.quantity{
            ahrs.worldLinearAccelX.toDouble().ofUnit(g)
        }
        override val yAcceleration: Acceleration by ACCELEROMETER_INPUTS.quantity{
            ahrs.worldLinearAccelY.toDouble().ofUnit(g)
        }
        override val zAcceleration: Acceleration by ACCELEROMETER_INPUTS.quantity{
            ahrs.worldLinearAccelZ.toDouble().ofUnit(g)
        }
    }

    public inner class Speedometer internal constructor(): ThreeAxisSpeedometer {
        override val xVelocity: Velocity by SPEEDOMETER_INPUTS.quantity{
            ahrs.velocityX.toDouble().ofUnit(meters / seconds)
        }
        override val yVelocity: Velocity by SPEEDOMETER_INPUTS.quantity{
            ahrs.velocityY.toDouble().ofUnit(meters / seconds)
        }
        override val zVelocity: Velocity by SPEEDOMETER_INPUTS.quantity{
            ahrs.velocityZ.toDouble().ofUnit(meters / seconds)
        }
    }
}