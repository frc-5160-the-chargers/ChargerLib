package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.rotations
import edu.wpi.first.wpilibj.DigitalSource
import edu.wpi.first.wpilibj.DutyCycle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.chargers.hardware.sensors.encoders.EncoderConfigurable
import frc.chargers.hardware.sensors.encoders.EncoderConfiguration
import frc.chargers.hardware.sensors.encoders.PositionEncoder

/**
 * An Adapter of WPILib's [DutyCycleEncoder] class; consists of REV through bore encoders, CTRE mag encoders.
 */
public class ChargerDutyCycleEncoder: DutyCycleEncoder, PositionEncoder,
    EncoderConfigurable<DutyCycleEncoderConfiguration> {

    public constructor(channel: Int): super(channel)
    public constructor(source: DigitalSource): super(source)
    public constructor(dutyCycle: DutyCycle): super(dutyCycle)

    override val angularPosition: Angle
        get() = get().ofUnit(rotations)

    override fun configure(configuration: DutyCycleEncoderConfiguration) {
        configuration.connectedFrequencyThreshold?.let { setConnectedFrequencyThreshold(it) }
        configuration.dutyCycleRange?.let{
            setDutyCycleRange(it.start,it.endInclusive)
        }
        configuration.positionOffset?.let{
            positionOffset = it
        }
    }

}

public data class DutyCycleEncoderConfiguration(
    var connectedFrequencyThreshold: Int? = null,
    var dutyCycleRange: ClosedRange<Double>? = null,
    var positionOffset: Double? = null
): EncoderConfiguration