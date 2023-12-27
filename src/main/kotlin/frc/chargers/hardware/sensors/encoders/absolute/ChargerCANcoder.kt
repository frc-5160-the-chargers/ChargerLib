package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.configs.CANcoderConfiguration as CTRECANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder as CTRECANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.sensors.encoders.ResettableTimestampedEncoder
import frc.chargers.hardware.sensors.encoders.TimestampedEncoder
import frc.chargers.utils.QuantityMeasurement


/**
 * Creates a [ChargerCANcoder] with inline configuration.
 */
public inline fun ChargerCANcoder(
    deviceId: Int,
    canBus: String = "",
    factoryDefault: Boolean = true,
    configure: CANcoderConfiguration.() -> Unit
): ChargerCANcoder = ChargerCANcoder(deviceId,canBus,factoryDefault).also{
    it.configure(CANcoderConfiguration().apply(configure))
}

/**
 * A wrapper for the CTRE's CANcoder class, with integration into chargerlib.
 *
 * @see CTRECANcoder
 * @see CANcoderConfiguration
 */
public class ChargerCANcoder(
    deviceId: Int,
    canBus: String = "",
    factoryDefault: Boolean = true
): CTRECANcoder(deviceId, canBus), ResettableTimestampedEncoder, HardwareConfigurable<CANcoderConfiguration> {

    init{
        if (factoryDefault){
            configurator.apply(CTRECANcoderConfiguration(),0.02)
            println("CANcoder has been factory defaulted.")
        }
    }

    public val absolute: TimestampedEncoder = AbsoluteEncoderAdaptor()
    
    override fun configure(configuration: CANcoderConfiguration) {
        val baseConfig = CTRECANcoderConfiguration()
        configurator.refresh(baseConfig)

        baseConfig.applyChanges(configuration)
        configurator.apply(baseConfig,0.050)


        configuration.positionUpdateFrequency?.let{
            position.setUpdateFrequency(it.inUnit(hertz))
            absolutePosition.setUpdateFrequency(it.inUnit(hertz))
        }

        configuration.velocityUpdateFrequency?.let{
            velocity.setUpdateFrequency(it.inUnit(hertz))
            unfilteredVelocity.setUpdateFrequency(it.inUnit(hertz))
        }

        configuration.filterVelocity?.let{ filterVelocity = it }

        println("CANcoder has been configured.")

    }

    private var filterVelocity: Boolean = true
    private val posSignal = position
    private val velSignal = if (filterVelocity) velocity else unfilteredVelocity

    override fun setZero(newZero: Angle){
        setPosition(newZero.inUnit(rotations))
    }

    override val timestampedAngularPosition: QuantityMeasurement<AngleDimension>
        get(){
            val statusSignal = posSignal.refresh()
            return QuantityMeasurement(
                value = statusSignal.value.ofUnit(rotations),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds)
            )
        }

    override val timestampedAngularVelocity: QuantityMeasurement<AngularVelocityDimension>
        get(){
            val statusSignal = velSignal.refresh(true)
            return QuantityMeasurement(
                value = statusSignal.value.ofUnit(rotations / seconds),
                timestamp = statusSignal.timestamp.time.ofUnit(seconds),
            )
        }


    private inner class AbsoluteEncoderAdaptor: TimestampedEncoder by this, HardwareConfigurable<CANcoderConfiguration> by this{

        private val absolutePosSignal = absolutePosition

        override val timestampedAngularPosition: QuantityMeasurement<AngleDimension>
            get(){
                val statusSignal = absolutePosSignal.refresh(true)
                return QuantityMeasurement(
                    value = statusSignal.value.ofUnit(rotations),
                    timestamp = statusSignal.timestamp.time.ofUnit(seconds)
                )
            }

        override val angularPosition: Angle
            get() = absolutePosSignal.refresh(true).value.ofUnit(rotations)
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
): HardwareConfiguration

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