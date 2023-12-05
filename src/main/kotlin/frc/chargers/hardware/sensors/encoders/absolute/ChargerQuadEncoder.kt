package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.hal.SimDevice
import edu.wpi.first.wpilibj.CounterBase.EncodingType
import edu.wpi.first.wpilibj.DigitalSource
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import edu.wpi.first.wpilibj.Encoder as QuadratureEncoder

/**
 * An adapter from the WPILib Encoder class to the ChargerLib Encoder interface.
 */
public class ChargerQuadEncoder:
    QuadratureEncoder, Encoder, HardwareConfigurable<QuadEncoderConfiguration> {

    override val angularPosition: Angle
        get() = get() * anglePerPulse
    override val angularVelocity: AngularVelocity
        get() = rate * anglePerPulse / 1.seconds

    override fun configure(configuration: QuadEncoderConfiguration) {
        configuration.samplesPerAverage?.let{ samplesToAverage = it }
        configuration.simDevice?.let{setSimDevice(it)}
        configuration.reverseDirection?.let{setReverseDirection(it)}
    }


    public val anglePerPulse: Angle
    /*
     * The various constructors for the Encoder wrapper.
     */
    public constructor(
        anglePerPulse: Angle,
        channelA: Int,
        channelB: Int,
        reverseDirection: Boolean = false,
        encodingType: EncodingType = EncodingType.k4X,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ) : super(channelA, channelB, reverseDirection, encodingType){
        this.anglePerPulse = anglePerPulse
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        pulsesPerRotation: Float,
        channelA: Int,
        channelB: Int,
        reverseDirection: Boolean = false,
        encodingType: EncodingType = EncodingType.k4X,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ) : super(channelA, channelB, reverseDirection, encodingType){
        this.anglePerPulse = (1/pulsesPerRotation.toDouble()).ofUnit(rotations)
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        anglePerPulse: Angle,
        channelA: Int,
        channelB: Int,
        indexChannel: Int,
        reverseDirection: Boolean = false,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ): super(channelA, channelB,indexChannel, reverseDirection){
        this.anglePerPulse = anglePerPulse
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        pulsesPerRotation: Float,
        channelA: Int,
        channelB: Int,
        indexChannel: Int,
        reverseDirection: Boolean = false,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ): super(channelA, channelB,indexChannel, reverseDirection){
        this.anglePerPulse = (1/pulsesPerRotation.toDouble()).ofUnit(rotations)
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        anglePerPulse: Angle,
        sourceA: DigitalSource,
        sourceB: DigitalSource,
        reverseDirection: Boolean = false,
        encodingType: EncodingType = EncodingType.k4X,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ): super(sourceA, sourceB, reverseDirection, encodingType){
        this.anglePerPulse = anglePerPulse
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        pulsesPerRotation: Float,
        sourceA: DigitalSource,
        sourceB: DigitalSource,
        reverseDirection: Boolean = false,
        encodingType: EncodingType = EncodingType.k4X,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ): super(sourceA, sourceB, reverseDirection, encodingType){
        this.anglePerPulse = (1/pulsesPerRotation.toDouble()).ofUnit(rotations)
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        anglePerPulse: Angle,
        sourceA: DigitalSource,
        sourceB: DigitalSource,
        indexSource: DigitalSource,
        reverseDirection: Boolean,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ): super(sourceA, sourceB, indexSource, reverseDirection){
        this.anglePerPulse = anglePerPulse
        configure(QuadEncoderConfiguration().apply(configure))
    }

    public constructor(
        pulsesPerRotation: Float,
        sourceA: DigitalSource,
        sourceB: DigitalSource,
        indexSource: DigitalSource,
        reverseDirection: Boolean,
        configure: QuadEncoderConfiguration.() -> Unit = {}
    ): super(sourceA, sourceB, indexSource, reverseDirection){
        this.anglePerPulse = (1/pulsesPerRotation.toDouble()).ofUnit(rotations)
        configure(QuadEncoderConfiguration().apply(configure))
    }

}

public data class QuadEncoderConfiguration(
    var samplesPerAverage: Int? = null,
    var simDevice: SimDevice? = null,
    var reverseDirection: Boolean? = null
): HardwareConfiguration