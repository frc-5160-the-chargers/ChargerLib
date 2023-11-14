package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.configs.CANcoderConfiguration as CTRECANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder as CTRECANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.chargers.hardware.inputdevices.warnIfInSimulation
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
    canBus: String = ""
): CTRECANcoder(deviceID, canBus), ResettableTimestampedEncoder, EncoderConfigurable<CANcoderConfiguration> {

    public var filterVelocity: Boolean = true
    public companion object{
        public inline operator fun invoke(
            deviceID: Int,
            canBus: String = "",
            configure: CANcoderConfiguration.() -> Unit = {}
        ): ChargerCANcoder = ChargerCANcoder(deviceID,canBus).also{
            it.configure(CANcoderConfiguration().apply(configure))
            warnIfInSimulation("ChargerCANcoder(ID = $deviceID)")
        }
    }

    public val absolute: TimestampedEncoder = AbsoluteEncoderAdaptor()

    private inner class AbsoluteEncoderAdaptor: TimestampedEncoder by this, EncoderConfigurable<CANcoderConfiguration> by this{
        override val timestampedAngularPosition: Measurement<Angle>
            get(){
                val statusSignal = absolutePosition
                //println(statusSignal.value.ofUnit(degrees))
                return Measurement(
                    value = statusSignal.value.ofUnit(rotations),
                    timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                    isValid = statusSignal.timestamp.isValid
                )
            }

        override val angularPosition: Angle
            get() = absolutePosition.value.ofUnit(rotations)
    }
    
    override fun configure(configuration: CANcoderConfiguration) {
        val baseConfig = CTRECANcoderConfiguration()
        configurator.refresh(baseConfig)

        configurator.apply(baseConfig.applyChanges(configuration))

        configuration.positionUpdateFrequency?.let{
            position.setUpdateFrequency(it.inUnit(hertz))
            absolutePosition.setUpdateFrequency(it.inUnit(hertz))
        }

        configuration.velocityUpdateFrequency?.let{
            velocity.setUpdateFrequency(it.inUnit(hertz))
            unfilteredVelocity.setUpdateFrequency(it.inUnit(hertz))
        }

        configuration.filterVelocity?.let{ filterVelocity = it }

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
            val statusSignal = if (filterVelocity) velocity else unfilteredVelocity
            return Measurement(
                value = statusSignal.value.ofUnit(rotations / seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
                isValid = statusSignal.timestamp.isValid
            )
        }
}


public data class CANcoderConfiguration(
    var futureProofConfigs: Boolean? = null,
    var sensorDirection: SensorDirectionValue? = null,
    var absoluteSensorRange: AbsoluteSensorRangeValue? = null,
    var magnetOffset: Angle? = null,
    var filterVelocity: Boolean? = null,

    var positionUpdateFrequency: Frequency? = null,
    var velocityUpdateFrequency: Frequency? = null
): EncoderConfiguration

public fun CTRECANcoderConfiguration.applyChanges(chargerConfig: CANcoderConfiguration): CTRECANcoderConfiguration{
    chargerConfig.futureProofConfigs?.let{
        FutureProofConfigs = it
    }

    MagnetSensor.apply{
        chargerConfig.sensorDirection?.let{SensorDirection = it}
        chargerConfig.absoluteSensorRange?.let{AbsoluteSensorRange = it}
        chargerConfig.magnetOffset?.let{MagnetOffset = it.inUnit(rotations)}
    }
    return this
}