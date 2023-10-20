package frc.chargers.hardware.swerve

import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain

/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public fun sparkMaxSwerveMotors(
    topLeft: ChargerCANSparkMax,
    topRight: ChargerCANSparkMax,
    bottomLeft: ChargerCANSparkMax,
    bottomRight: ChargerCANSparkMax,
    configure: SparkMaxConfiguration.() -> Unit = {}
): SwerveMotors = SwerveMotors(
    topLeft, topRight, bottomLeft, bottomRight,SparkMaxConfiguration().apply(configure)
)

/**
 * Constructs an instance of [SwerveMotors] with TalonFX motor controllers.
 */
public fun talonFXSwerveMotors(
    topLeft: ChargerTalonFX,
    topRight: ChargerTalonFX,
    bottomLeft: ChargerTalonFX,
    bottomRight: ChargerTalonFX,
    configure: TalonFXConfiguration.() -> Unit = {}
): SwerveMotors = SwerveMotors(
    topLeft, topRight, bottomLeft, bottomRight,TalonFXConfiguration().apply(configure)
)



/**
 * A Helper class to store a group of motors needed for an [EncoderHolonomicDrivetrain];
 * these can be either for turning or driving.
 */
public open class SwerveMotors(
    public open val topLeft: EncoderMotorController,
    public open val topRight: EncoderMotorController,
    public open val bottomLeft: EncoderMotorController,
    public open val bottomRight: EncoderMotorController
){
    /**
     * This companion object stores an [invoke] function, which creates a [SwerveMotors] instance
     * where all the motors are configured on initialization.
     */
    public companion object{
        public operator fun <M, C: MotorConfiguration> invoke(
            topLeft: M,
            topRight: M,
            bottomLeft: M,
            bottomRight: M,
            configuration: C? = null
        ): SwerveMotors where M: EncoderMotorController, M: MotorConfigurable<C> =
            SwerveMotors(
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