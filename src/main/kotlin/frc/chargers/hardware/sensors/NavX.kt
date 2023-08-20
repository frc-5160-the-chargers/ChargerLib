package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.kauailabs.navx.frc.AHRS
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope
import frc.chargers.utils.math.units.g

public class NavX(public val ahrs: AHRS = AHRS()) : IMU {
    init {
        reset()
    }

    override fun reset() {
        ahrs.reset()
    }

    override val heading: Angle
        get() = -ahrs.fusedHeading.toDouble().ofUnit(degrees) // Negative sign because the navX reports clockwise as positive
                                                              // whereas we want counterclockwise to be positive

    public val altitude: Distance?
        get() = if (ahrs.isAltitudeValid) ahrs.altitude.toDouble().ofUnit(meters) else null

    public val firmwareVersion: String get() = ahrs.firmwareVersion

    public val isConnected: Boolean
        get() = ahrs.isConnected

    override val gyroscope: Gyroscope = Gyroscope()
    override val compass: Compass = Compass()
    override val accelerometer: Accelerometer = Accelerometer()
    override val speedometer: Speedometer = Speedometer()

    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope, HeadingProvider {
        public override val yaw: Angle
            get() = ahrs.yaw.toDouble().ofUnit(degrees)
        override val pitch: Angle
            get() = ahrs.pitch.toDouble().ofUnit(degrees)
        override val roll: Angle
            get() = ahrs.roll.toDouble().ofUnit(degrees)
        override val heading: Angle
            get() = ahrs.angle.ofUnit(degrees) // Negative sign because the navX reports clockwise as positive
                                               // whereas we want counterclockwise to be positive
    }

    public inner class Compass internal constructor(): HeadingProvider {
        public override val heading: Angle
            get() = -ahrs.compassHeading.toDouble().ofUnit(degrees) // Negative sign because the navX reports clockwise as positive
                                                                    // whereas we want counterclockwise to be positive
    }

    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer{
        override val xAcceleration: Acceleration
            get() = ahrs.worldLinearAccelX.toDouble().ofUnit(g)
        override val yAcceleration: Acceleration
            get() = ahrs.worldLinearAccelY.toDouble().ofUnit(g)
        override val zAcceleration: Acceleration
            get() = ahrs.worldLinearAccelZ.toDouble().ofUnit(g)
    }

    public inner class Speedometer: ThreeAxisSpeedometer{
        override val xVelocity: Velocity
            get() = ahrs.velocityX.toDouble().ofUnit(meters / seconds)
        override val yVelocity: Velocity
            get() = ahrs.velocityY.toDouble().ofUnit(meters / seconds)
        override val zVelocity: Velocity
            get() = ahrs.velocityZ.toDouble().ofUnit(meters / seconds)
    }
}

