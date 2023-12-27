package frc.chargers.hardware.subsystemutils.swervedrive

import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import kotlin.internal.LowPriorityInOverloadResolution

/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public fun sparkMaxSwerveMotors(
    topLeft: ChargerCANSparkMax,
    topRight: ChargerCANSparkMax,
    bottomLeft: ChargerCANSparkMax,
    bottomRight: ChargerCANSparkMax,
    configure: SparkMaxConfiguration.() -> Unit = {}
): SwerveMotors = OnboardPIDSwerveMotors(
    topLeft, topRight, bottomLeft, bottomRight, SparkMaxConfiguration().apply(configure)
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
): SwerveMotors = OnboardPIDSwerveMotors(
    topLeft, topRight, bottomLeft, bottomRight, TalonFXConfiguration().apply(configure)
)


@LowPriorityInOverloadResolution
public fun <M, C: HardwareConfiguration> OnboardPIDSwerveMotors(
    topLeft: M,
    topRight: M,
    bottomLeft: M,
    bottomRight: M,
    configuration: C? = null
): OnboardPIDSwerveMotors where M: SmartEncoderMotorController, M: HardwareConfigurable<C> =
    OnboardPIDSwerveMotors(
        topLeft = topLeft.apply{
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



@LowPriorityInOverloadResolution
public fun <M, C: HardwareConfiguration> SwerveMotors(
    topLeft: M,
    topRight: M,
    bottomLeft: M,
    bottomRight: M,
    configuration: C? = null
): SwerveMotors where M: EncoderMotorController, M: HardwareConfigurable<C> =
    SwerveMotors(
        topLeft = topLeft.apply{
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


public class OnboardPIDSwerveMotors(
    override val topLeft: SmartEncoderMotorController,
    override val topRight: SmartEncoderMotorController,
    override val bottomLeft: SmartEncoderMotorController,
    override val bottomRight: SmartEncoderMotorController
): SwerveMotors(topLeft,topRight,bottomLeft,bottomRight)

/**
 * A Helper class to store a group of motors needed for an [EncoderHolonomicDrivetrain];
 * these can be either for turning or driving.
 */
public open class SwerveMotors(
    public open val topLeft: EncoderMotorController,
    public open val topRight: EncoderMotorController,
    public open val bottomLeft: EncoderMotorController,
    public open val bottomRight: EncoderMotorController
)