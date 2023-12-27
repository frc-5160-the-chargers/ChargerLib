package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.sensors.ThreeAxisAccelerometer
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.utils.math.units.g
import com.ctre.phoenix6.configs.Pigeon2Configuration as CTREPigeon2Configuration

/**
 * Creates a [ChargerPigeon2] with inline configuration.
 */
public fun ChargerPigeon2(
    canId: Int,
    canBus: String = "rio",
    configure: Pigeon2Configuration.() -> Unit
): ChargerPigeon2 = ChargerPigeon2(canId,canBus).also{
    val config = Pigeon2Configuration().apply(configure)
    it.configure(config)
}


/**
 * A wrapper around the [Pigeon2] class from CTRE,
 * with units support and interface implementation.
 *
 * @see Pigeon2Configuration
 */
public class ChargerPigeon2(
    canId: Int,
    canBus: String = "rio"
): Pigeon2(canId, canBus), ZeroableHeadingProvider, HardwareConfigurable<Pigeon2Configuration>{

    /**
     * The gyroscope of the Pigeon; contains yaw, pitch, and roll data
     * as well as the recorded rate(s) for those values.
     */
    public val gyroscope: Gyroscope = Gyroscope()

    /**
     * The accelerometer of the Pigeon; contains x, y, and z acceleration.
     */
    public val accelerometer: Accelerometer = Accelerometer()

    /**
     * The heading of the Pigeon; equivalent to yaw.
     */
    override val heading: Angle by ImuLog.quantity{ gyroscope.yaw }

    /**
     * Zeroes the heading of the Pigeon.
     */
    override fun zeroHeading() { reset() }

    override fun configure(configuration: Pigeon2Configuration) {
        val ctreConfig = CTREPigeon2Configuration()
        configurator.refresh(ctreConfig)
        ctreConfig.applyChanges(configuration)
        configurator.apply(ctreConfig)
    }

    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope {
        private val yawSignal = getYaw()
        private val rollSignal = getRoll()
        private val pitchSignal = getPitch()

        private val yawRateSignal = angularVelocityZ
        private val pitchRateSignal = angularVelocityY
        private val rollRateSignal = angularVelocityX

        private var simPreviousYaw = Angle(0.0)

        override val yaw: Angle by GyroLog.quantity{
            if (isReal()) yawSignal.refresh(true).value.ofUnit(degrees) else getSimHeading()
        }

        override val pitch: Angle by GyroLog.quantity{
            if (isReal()) pitchSignal.refresh(true).value.ofUnit(degrees) else Angle(0.0)
        }

        override val roll: Angle by GyroLog.quantity{
            if (isReal()) rollSignal.refresh(true).value.ofUnit(degrees) else Angle(0.0)
        }

        public val yawRate: AngularVelocity by GyroLog.quantity{
            if (isReal()){
                yawRateSignal.refresh(true).value.ofUnit(degrees/seconds)
            }else{
                val currH = getSimHeading()
                ((currH - simPreviousYaw) / ChargerRobot.LOOP_PERIOD).also{
                    simPreviousYaw = currH
                }
            }
        }

        public val pitchRate: AngularVelocity by GyroLog.quantity{
            if (isReal()) pitchRateSignal.refresh(true).value.ofUnit(degrees/seconds) else AngularVelocity(0.0)
        }

        public val rollRate: AngularVelocity by GyroLog.quantity{
            if (isReal()) rollRateSignal.refresh(true).value.ofUnit(degrees/seconds) else AngularVelocity(0.0)
        }

    }

    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer {
        private val xAccelSignal = accelerationX
        private val yAccelSignal = accelerationY
        private val zAccelSignal = accelerationZ

        override val xAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) xAccelSignal.refresh(true).value.ofUnit(g) else Acceleration(0.0)
        }

        override val yAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) yAccelSignal.refresh(true).value.ofUnit(g) else Acceleration(0.0)
        }

        override val zAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) zAccelSignal.refresh(true).value.ofUnit(g) else Acceleration(0.0)
        }
    }

}

/**
 * A configuration class for the [ChargerPigeon2].
 */
public data class Pigeon2Configuration(
    var futureProofConfigs: Boolean? = null,
    var gyroScalarX: Angle? = null,
    var gyroScalarY: Angle? = null,
    var gyroScalarZ: Angle? = null,
    var mountPosePitch: Angle? = null,
    var mountPoseYaw: Angle? = null,
    var mountPoseRoll: Angle? = null,
    var disableNoMotionCalibration: Boolean? = null,
    var disableTemperatureCompensation: Boolean? = null,
    var enableCompass: Boolean? = null
): HardwareConfiguration

public fun CTREPigeon2Configuration.applyChanges(config: Pigeon2Configuration): CTREPigeon2Configuration{
    config.futureProofConfigs?.let{ FutureProofConfigs = it }
    GyroTrim.apply{
        config.gyroScalarX?.let{ GyroScalarX = it.inUnit(degrees) }
        config.gyroScalarY?.let{ GyroScalarY = it.inUnit(degrees) }
        config.gyroScalarZ?.let{ GyroScalarZ = it.inUnit(degrees) }
    }

    MountPose.apply{
        config.mountPosePitch?.let{ MountPosePitch = it.inUnit(degrees) }
        config.mountPoseYaw?.let{ MountPoseYaw = it.inUnit(degrees) }
        config.mountPoseRoll?.let{ MountPoseRoll = it.inUnit(degrees) }
    }

    Pigeon2Features.apply{
        config.disableTemperatureCompensation?.let{DisableTemperatureCompensation = it}
        config.disableNoMotionCalibration?.let{DisableNoMotionCalibration = it}
        config.enableCompass?.let{EnableCompass = it}
    }

    return this
}