package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.absolute.CANcoderConfiguration
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.withOffset

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
    val config = CANcoderConfiguration().apply(configure)
    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return if (useAbsoluteSensor){
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