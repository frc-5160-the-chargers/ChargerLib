package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.kauailabs.navx.frc.AHRS
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope
import frc.chargers.utils.math.units.g

public class NavX(public val ahrs: AHRS = AHRS()) : HeadingProvider {
    init {
        reset()
    }

    public fun reset() {
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

    public val gyroscope: Gyroscope = Gyroscope()
    public val compass: Compass = Compass()
    public val accelerometer: Accelerometer = Accelerometer()

    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope, HeadingProvider {
        // used to "calibrate" the values to zero
        public var yawCalibration: Angle = 0.0.degrees
        public var pitchCalibration: Angle = 0.0.degrees
        public var rollCalibration: Angle = 0.0.degrees
        public var headingCalibration: Angle = 0.0.degrees

        public fun calibrateYaw(target: Angle = 0.0.degrees){
            yawCalibration = yaw - target
        }

        public fun calibratePitch(target: Angle = 0.0.degrees){
            pitchCalibration = pitch - target
        }

        public fun calibrateRoll(target: Angle = 0.0.degrees){
            rollCalibration = roll - target
        }

        public fun calibrateHeading(target: Angle = 0.0.degrees){
            headingCalibration = heading - target
        }

        public override val yaw: Angle
            get() = ahrs.yaw.toDouble().ofUnit(degrees) - yawCalibration
        override val pitch: Angle
            get() = ahrs.pitch.toDouble().ofUnit(degrees) - pitchCalibration
        override val roll: Angle
            get() = ahrs.roll.toDouble().ofUnit(degrees) - rollCalibration
        override val heading: Angle
            get() = ahrs.angle.ofUnit(degrees) - headingCalibration
    }

    public inner class Compass internal constructor(): HeadingProvider {
        public override val heading: Angle
            get() = -ahrs.compassHeading.toDouble().ofUnit(degrees) // Negative sign because the navX reports clockwise as positive
                                                                    // whereas we want counterclockwise to be positive
    }

    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer, ThreeAxisSpeedometer {
        override val xAcceleration: Acceleration
            get() = ahrs.worldLinearAccelX.toDouble().ofUnit(g)
        override val yAcceleration: Acceleration
            get() = ahrs.worldLinearAccelY.toDouble().ofUnit(g)
        override val zAcceleration: Acceleration
            get() = ahrs.worldLinearAccelZ.toDouble().ofUnit(g)
        override val xVelocity: Velocity
            get() = ahrs.velocityX.toDouble().ofUnit(meters / seconds)
        override val yVelocity: Velocity
            get() = ahrs.velocityY.toDouble().ofUnit(meters / seconds)
        override val zVelocity: Velocity
            get() = ahrs.velocityZ.toDouble().ofUnit(meters / seconds)
    }
}