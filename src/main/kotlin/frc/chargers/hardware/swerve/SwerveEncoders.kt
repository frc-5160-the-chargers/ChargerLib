package frc.chargers.hardware.swerve

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.encoders.EncoderConfigurable
import frc.chargers.hardware.sensors.encoders.EncoderConfiguration
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.absolute.CANcoderConfiguration
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder

public fun swerveCANcoders(
    topLeft: ChargerCANcoder,
    topRight: ChargerCANcoder,
    bottomLeft: ChargerCANcoder,
    bottomRight: ChargerCANcoder,
    useAbsoluteSensor: Boolean,
    configure: CANcoderConfiguration.() -> Unit = {}
): SwerveEncoders{
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


public data class SwerveEncoders(
    val topLeft: PositionEncoder,
    val topRight: PositionEncoder,
    val bottomLeft: PositionEncoder,
    val bottomRight: PositionEncoder
){
    public companion object{
        public operator fun <E, C: EncoderConfiguration> invoke(
            topLeft: E,
            topRight: E,
            bottomLeft: E,
            bottomRight: E,
            configuration: C? = null
        ): SwerveEncoders where E: PositionEncoder, E: EncoderConfigurable<C> = 
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
    }
}