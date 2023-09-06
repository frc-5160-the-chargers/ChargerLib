package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.CANcoderConfiguration as CTRECANCoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder as CTRECANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.hardware.sensors.encoders.TimestampedEncoder
import frc.chargers.utils.Measurement


/**
 * A wrapper for the [CTRECANcoder] class, with integration into chargerlib.
 */
public class ChargerCANCoder(
    deviceID: Int,
    canBus: String? = null
): CTRECANcoder(deviceID, canBus), TimestampedEncoder, ResettableEncoder, MotorConfigurable<CANCoderConfiguration> {

    public companion object{
        public inline operator fun invoke(
            deviceID: Int,
            canBus: String? = null,
            configure: CANCoderConfiguration.() -> Unit = {}
        ): ChargerCANCoder = ChargerCANCoder(deviceID,canBus).also{
            it.configure(CANCoderConfiguration().apply(configure))
        }
    }

    public val absolute: TimestampedEncoder = AbsoluteEncoderAdaptor()

    private inner class AbsoluteEncoderAdaptor: TimestampedEncoder by this{
        override val timestampedAngularPosition: Measurement<Angle>
            get(){
                val statusSignal = absolutePosition
                return Measurement(
                    value = statusSignal.value.ofUnit(rotations),
                    timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                    isValid = statusSignal.timestamp.isValid
                )
            }
    }
    
    override fun configure(configuration: CANCoderConfiguration) {
        configurator.apply(configuration.toCTRECANCoderConfiguration())
    }

    override fun setZero(newZero: Angle){
        setPosition(newZero.inUnit(rotations))
    }
    override val timestampedAngularPosition: Measurement<Angle>
        get(){
            val statusSignal = position
            return Measurement(
                value = statusSignal.value.ofUnit(rotations),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = statusSignal.timestamp.isValid
            )
        }

    override val timestampedAngularVelocity: Measurement<AngularVelocity>
        get(){
            val statusSignal = velocity
            return Measurement(
                value = statusSignal.value.ofUnit(rotations / seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = absolutePosition.timestamp.isValid
            )
        }
}

private val blankConfig = CTRECANCoderConfiguration()

public data class CANCoderConfiguration(
    var futureProofConfigs: Boolean = true,
    var sensorDirection: SensorDirectionValue = blankConfig.MagnetSensor.SensorDirection,
    var absoluteSensorRange: AbsoluteSensorRangeValue = blankConfig.MagnetSensor.AbsoluteSensorRange,
    var magnetOffset: Angle = Angle(0.0),
): MotorConfiguration{
    public fun toCTRECANCoderConfiguration(): CTRECANCoderConfiguration = CTRECANCoderConfiguration().apply{
        FutureProofConfigs = futureProofConfigs
        MagnetSensor = MagnetSensorConfigs().apply{
            SensorDirection = sensorDirection
            AbsoluteSensorRange = absoluteSensorRange
            MagnetOffset = magnetOffset.inUnit(rotations)
        }
    }
}