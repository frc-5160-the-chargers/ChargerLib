package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.absolute.CANcoderConfiguration
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.withOffset
import kotlin.internal.LowPriorityInOverloadResolution

/**
 * Constructs an instance of [SwerveEncoders] with CTRE CANcoders.
 */
public inline fun swerveCANcoders(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    useAbsoluteSensor: Boolean,
    configure: CANcoderConfiguration.() -> Unit = {}
): SwerveEncoders = swerveCANcoders(
    ChargerCANcoder(topLeftId),
    ChargerCANcoder(topRightId),
    ChargerCANcoder(bottomLeftId),
    ChargerCANcoder(bottomRightId),
    useAbsoluteSensor, configure
)

/**
 * Constructs an instance of [SwerveEncoders] with CTRE CANcoders.
 */
public inline fun swerveCANcoders(
    topLeft: ChargerCANcoder,
    topRight: ChargerCANcoder,
    bottomLeft: ChargerCANcoder,
    bottomRight: ChargerCANcoder,
    useAbsoluteSensor: Boolean,
    configure: CANcoderConfiguration.() -> Unit = {}
): SwerveEncoders {
    topLeft.configure(CANcoderConfiguration().apply(configure))
    topRight.configure(CANcoderConfiguration().apply(configure))
    bottomLeft.configure(CANcoderConfiguration().apply(configure))
    bottomRight.configure(CANcoderConfiguration().apply(configure))

    return if(useAbsoluteSensor){
        SwerveEncoders(
            topLeft.absolute,
            topRight.absolute,
            bottomLeft.absolute,
            bottomRight.absolute
        )
    }else{
        SwerveEncoders(
            topLeft,
            topRight,
            bottomLeft,
            bottomRight
        )
    }

}

@LowPriorityInOverloadResolution
public fun <E, C: HardwareConfiguration> SwerveEncoders(
    topLeft: E,
    topRight: E,
    bottomLeft: E,
    bottomRight: E,
    configuration: C? = null
): SwerveEncoders where E: PositionEncoder, E: HardwareConfigurable<C> =
    SwerveEncoders(
        topLeft.apply{
            if(configuration != null){
                configure(configuration)
            }
        },
        topRight.apply{
            if(configuration != null){
                configure(configuration)
            }
        },
        bottomLeft.apply{
            if(configuration != null){
                configure(configuration)
            }
        },
        bottomRight.apply{
            if(configuration != null){
                configure(configuration)
            }
        }
    )


public data class SwerveEncoders(
    val topLeft: PositionEncoder,
    val topRight: PositionEncoder,
    val bottomLeft: PositionEncoder,
    val bottomRight: PositionEncoder
){

    public fun withOffsets(
        topLeftZero: Angle,
        topRightZero: Angle,
        bottomLeftZero: Angle,
        bottomRightZero: Angle
    ): SwerveEncoders = SwerveEncoders(
        topLeft = topLeft.withOffset(topLeftZero),
        topRight = topRight.withOffset(topRightZero),
        bottomLeft = bottomLeft.withOffset(bottomLeftZero),
        bottomRight = bottomRight.withOffset(bottomRightZero),
    )

}