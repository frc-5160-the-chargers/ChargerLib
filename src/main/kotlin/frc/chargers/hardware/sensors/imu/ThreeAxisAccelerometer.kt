package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Acceleration

/**
 * Represents a generic Accelerometer, with acceleration measurements for the x, y, and z axes.
 */
public interface ThreeAxisAccelerometer {
    public val xAcceleration: Acceleration
    public val yAcceleration: Acceleration
    public val zAcceleration: Acceleration


    public companion object{
        /**
         * Inline syntax to create a generic [ThreeAxisAccelerometer].
         */
        public inline operator fun invoke(
            crossinline getXAccel: () -> Acceleration,
            crossinline getYAccel: () -> Acceleration,
            crossinline getZAccel: () -> Acceleration
        ): ThreeAxisAccelerometer = object: ThreeAxisAccelerometer {
            override val xAcceleration: Acceleration get() = getXAccel()
            override val yAcceleration: Acceleration get() = getYAccel()
            override val zAcceleration: Acceleration get() = getZAccel()
        }
    }
}