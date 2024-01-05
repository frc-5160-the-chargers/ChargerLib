package frc.chargers.hardware.subsystems.swervedrive

import com.revrobotics.CANSparkMaxLowLevel
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.motorcontrol.rev.brushedSparkMax
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.Encoder


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
): SwerveMotors<ChargerCANSparkMax> = when (type){
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
): SwerveMotors<ChargerTalonFX> = talonFXSwerveMotors(
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
): SwerveMotors<ChargerCANSparkMax> {
    val config = SparkMaxConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return SwerveMotors(
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
): SwerveMotors<ChargerTalonFX>{
    val config = TalonFXConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return SwerveMotors(
        topLeft, topRight, bottomLeft, bottomRight,
    )
}

/**
 * A Helper class to store a group of motors needed for an [EncoderHolonomicDrivetrain];
 * these can be either for turning or driving.
 */
public data class SwerveMotors<out M: EncoderMotorController>(
    val topLeft: M,
    val topRight: M,
    val bottomLeft: M,
    val bottomRight: M
){
    // inline reified allows for runtime checks of whether or not the motors
    // are a specified type.
    public inline fun <reified T: EncoderMotorController> containsMotors(): Boolean =
        topLeft is T && topRight is T && bottomLeft is T && bottomRight is T


    public fun getEncoders(): SwerveEncoders<Encoder> =
        SwerveEncoders(
            topLeft.encoder,
            topRight.encoder,
            bottomLeft.encoder,
            bottomRight.encoder
        )

    public fun forEach(predicate: (M) -> Unit){
        predicate(topLeft)
        predicate(topRight)
        predicate(bottomLeft)
        predicate(bottomRight)
    }
}