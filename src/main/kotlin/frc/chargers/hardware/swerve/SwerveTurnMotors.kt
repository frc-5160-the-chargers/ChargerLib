package frc.chargers.hardware.swerve
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.FeedbackMotorController
import frc.chargers.hardware.motorcontrol.MotorConfigurable
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain


public fun sparkMaxTurnMotors(
    topLeft: ChargerCANSparkMax,
    topRight: ChargerCANSparkMax,
    bottomLeft: ChargerCANSparkMax,
    bottomRight: ChargerCANSparkMax,
    onboardPIDEnabled: Boolean = false,
    configure: SparkMaxConfiguration.() -> Unit = {}
): OnboardPIDSwerveTurnMotors = OnboardPIDSwerveTurnMotors(
    topLeft, topRight, bottomLeft, bottomRight,onboardPIDEnabled,SparkMaxConfiguration().apply(configure)
)

public fun talonFXTurnMotors(
    topLeft: ChargerTalonFX,
    topRight: ChargerTalonFX,
    bottomLeft: ChargerTalonFX,
    bottomRight: ChargerTalonFX,
    onboardPIDEnabled: Boolean = false,
    configure: TalonFXConfiguration.() -> Unit = {}
): OnboardPIDSwerveTurnMotors = OnboardPIDSwerveTurnMotors(
    topLeft, topRight, bottomLeft, bottomRight,onboardPIDEnabled,TalonFXConfiguration().apply(configure)
)


/**
 * A Helper class to store all the Turn motors needed for an [EncoderHolonomicDrivetrain],
 * where all the motors support onboard PID control.
 */
public class OnboardPIDSwerveTurnMotors(
    override val topLeft: FeedbackMotorController,
    override val topRight: FeedbackMotorController,
    override val bottomLeft: FeedbackMotorController,
    override val bottomRight: FeedbackMotorController,
    public val onboardPIDEnabled: Boolean = true
): SwerveTurnMotors(topLeft,topRight,bottomLeft,bottomRight){
    /**
     * This companion object stores an [invoke] function, which creates a [OnboardPIDSwerveTurnMotors] instance
     * where all the motors are configured on initialization.
     */
    public companion object{
        public operator fun <M, C: MotorConfiguration>invoke(
            topLeft: M,
            topRight: M,
            bottomLeft: M,
            bottomRight: M,
            onboardPIDEnabled: Boolean = true,
            configuration: C? = null
        ): OnboardPIDSwerveTurnMotors where M: FeedbackMotorController, M: MotorConfigurable<C> =
            OnboardPIDSwerveTurnMotors(
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
                },
                onboardPIDEnabled
            )
    }
}


/**
 * A Helper class to store all the Turn motors needed for an [EncoderHolonomicDrivetrain]
 */
public open class SwerveTurnMotors(
    public open val topLeft: EncoderMotorController,
    public open val topRight: EncoderMotorController,
    public open val bottomLeft: EncoderMotorController,
    public open val bottomRight: EncoderMotorController
){
    /**
     * This companion object stores an [invoke] function, which creates a [SwerveTurnMotors] instance
     * where all the motors are configured on initialization.
     */
    public companion object{
        public operator fun <M, C: MotorConfiguration>invoke(
            topLeft: M,
            topRight: M,
            bottomLeft: M,
            bottomRight: M,
            configuration: C? = null
        ): SwerveTurnMotors where M: EncoderMotorController, M: MotorConfigurable<C> =
            SwerveTurnMotors(
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