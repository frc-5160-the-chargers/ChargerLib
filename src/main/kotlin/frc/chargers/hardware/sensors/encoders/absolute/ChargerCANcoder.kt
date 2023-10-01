package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.CANcoderConfiguration as CTRECANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder as CTRECANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.chargers.hardware.sensors.encoders.EncoderConfigurable
import frc.chargers.hardware.sensors.encoders.EncoderConfiguration
import frc.chargers.hardware.sensors.encoders.ResettableTimestampedEncoder
import frc.chargers.hardware.sensors.encoders.TimestampedEncoder
import frc.chargers.utils.Measurement


/**
 * A wrapper for the CTRE's CANCoder class, with integration into chargerlib.
 *
 * @see CTRECANcoder
 * @see CANcoderConfiguration
 */
public class ChargerCANcoder(
    deviceID: Int,
    canBus: String? = null
): CTRECANcoder(deviceID, canBus), ResettableTimestampedEncoder, EncoderConfigurable<CANcoderConfiguration> {

    public companion object{
        public inline operator fun invoke(
            deviceID: Int,
            canBus: String? = null,
            configure: CANcoderConfiguration.() -> Unit = {}
        ): ChargerCANcoder = ChargerCANcoder(deviceID,canBus).also{
            it.configure(CANcoderConfiguration().apply(configure))
        }
    }

    public val absolute: TimestampedEncoder = AbsoluteEncoderAdaptor()

    private inner class AbsoluteEncoderAdaptor: TimestampedEncoder by this, EncoderConfigurable<CANcoderConfiguration> by this{
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
    
    override fun configure(configuration: CANcoderConfiguration) {
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

private val blankConfig = CTRECANcoderConfiguration()

public data class CANcoderConfiguration(
    var futureProofConfigs: Boolean = true,
    var sensorDirection: SensorDirectionValue = blankConfig.MagnetSensor.SensorDirection,
    var absoluteSensorRange: AbsoluteSensorRangeValue = blankConfig.MagnetSensor.AbsoluteSensorRange,
    var magnetOffset: Angle = Angle(0.0),
): EncoderConfiguration{
    public fun toCTRECANCoderConfiguration(): CTRECANcoderConfiguration = CTRECANcoderConfiguration().apply{
        FutureProofConfigs = futureProofConfigs
        MagnetSensor = MagnetSensorConfigs().apply{
            SensorDirection = sensorDirection
            AbsoluteSensorRange = absoluteSensorRange
            MagnetOffset = magnetOffset.inUnit(rotations)
        }
    }
}