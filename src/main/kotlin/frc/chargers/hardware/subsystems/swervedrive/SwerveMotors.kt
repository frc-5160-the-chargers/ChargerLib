package frc.chargers.hardware.subsystems.swervedrive

import com.revrobotics.CANSparkMaxLowLevel
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.motorcontrol.rev.brushedSparkMax
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax


/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public inline fun sparkMaxSwerveMotors(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    type: CANSparkMaxLowLevel.MotorType = CANSparkMaxLowLevel.MotorType.kBrushless,
    configure: SparkMaxConfiguration.() -> Unit = {}
): SwerveMotors = when (type){
    CANSparkMaxLowLevel.MotorType.kBrushless -> sparkMaxSwerveMotors(
        neoSparkMax(topLeftId),
        neoSparkMax(topRightId),
        neoSparkMax(bottomLeftId),
        neoSparkMax(bottomRightId),
        configure
    )
    CANSparkMaxLowLevel.MotorType.kBrushed -> sparkMaxSwerveMotors(
        brushedSparkMax(topLeftId),
        brushedSparkMax(topRightId),
        brushedSparkMax(bottomLeftId),
        brushedSparkMax(bottomRightId),
        configure
    )
}


/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public inline fun talonFXSwerveMotors(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    configure: TalonFXConfiguration.() -> Unit = {}
): SwerveMotors = talonFXSwerveMotors(
    falcon(topLeftId),
    falcon(topRightId),
    falcon(bottomLeftId),
    falcon(bottomRightId),
    configure
)

/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public inline fun sparkMaxSwerveMotors(
    topLeft: ChargerCANSparkMax,
    topRight: ChargerCANSparkMax,
    bottomLeft: ChargerCANSparkMax,
    bottomRight: ChargerCANSparkMax,
    configure: SparkMaxConfiguration.() -> Unit = {}
): SwerveMotors{
    val config = SparkMaxConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return OnboardPIDSwerveMotors(
        topLeft, topRight, bottomLeft, bottomRight,
    )
}


/**
 * Constructs an instance of [SwerveMotors] with TalonFX motor controllers.
 */
public inline fun talonFXSwerveMotors(
    topLeft: ChargerTalonFX,
    topRight: ChargerTalonFX,
    bottomLeft: ChargerTalonFX,
    bottomRight: ChargerTalonFX,
    configure: TalonFXConfiguration.() -> Unit = {}
): SwerveMotors{
    val config = TalonFXConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return OnboardPIDSwerveMotors(
        topLeft, topRight, bottomLeft, bottomRight,
    )
}

/**
 * A Helper class to store a group of motors needed for an [EncoderHolonomicDrivetrain],
 * which can run onboard PID control.
 */
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