package frc.chargers.hardware.swerve

import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain


public fun sparkMaxDriveMotors(
    topLeft: ChargerCANSparkMax,
    topRight: ChargerCANSparkMax,
    bottomLeft: ChargerCANSparkMax,
    bottomRight: ChargerCANSparkMax,
    configure: SparkMaxConfiguration.() -> Unit = {}
): SwerveDriveMotors = SwerveDriveMotors(
    topLeft, topRight, bottomLeft, bottomRight,SparkMaxConfiguration().apply(configure)
)

public fun talonFXDriveMotors(
    topLeft: ChargerTalonFX,
    topRight: ChargerTalonFX,
    bottomLeft: ChargerTalonFX,
    bottomRight: ChargerTalonFX,
    configure: TalonFXConfiguration.() -> Unit = {}
): SwerveDriveMotors = SwerveDriveMotors(
    topLeft, topRight, bottomLeft, bottomRight,TalonFXConfiguration().apply(configure)
)



/**
 * A Helper class to store all the drive motors needed for an [EncoderHolonomicDrivetrain]
 */
public open class SwerveDriveMotors(
    public open val topLeft: EncoderMotorController,
    public open val topRight: EncoderMotorController,
    public open val bottomLeft: EncoderMotorController,
    public open val bottomRight: EncoderMotorController
){
    /**
     * This companion object stores an [invoke] function, which creates a [SwerveDriveMotors] instance
     * where all the motors are configured on initialization.
     */
    public companion object{
        public operator fun <M, C: MotorConfiguration>invoke(
            topLeft: M,
            topRight: M,
            bottomLeft: M,
            bottomRight: M,
            configuration: C? = null
        ): SwerveDriveMotors where M: EncoderMotorController, M: MotorConfigurable<C> =
            SwerveDriveMotors(
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