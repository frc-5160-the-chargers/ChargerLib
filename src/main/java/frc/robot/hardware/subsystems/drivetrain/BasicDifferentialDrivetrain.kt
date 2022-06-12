package frc.robot.hardware.subsystems.drivetrain

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.interfaces.MotorConfigurable
import frc.robot.hardware.interfaces.MotorConfiguration
import frc.robot.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.robot.hardware.motorcontrol.SparkConfiguration
import frc.robot.hardware.motorcontrol.ctre.TalonSRXConfiguration

/**
 * A convenience function to create a [BasicDifferentialDrivetrain]
 * using Spark motor controllers.
 */
public inline fun sparkDrivetrain(
    leftMotors: EncoderMotorControllerGroup<SparkConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkConfiguration>,
    invertMotors: Boolean = false,
    configure: SparkConfiguration.() -> Unit = {}
): BasicDifferentialDrivetrain =
    BasicDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, SparkConfiguration().apply(configure))

/**
 * A convenience function to create a [BasicDifferentialDrivetrain]
 * using Talon SRX motor controllers.
 */
public inline fun talonSRXDrivetrain(
    leftMotors: EncoderMotorControllerGroup<TalonSRXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonSRXConfiguration>,
    invertMotors: Boolean = false,
    configure: TalonSRXConfiguration.() -> Unit = {}
): BasicDifferentialDrivetrain =
    BasicDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, TalonSRXConfiguration().apply(configure))

/**
 * A convenience function to create a [BasicDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : MotorConfiguration, M> BasicDifferentialDrivetrain(
    leftMotors: M,
    rightMotors: M,
    invertMotors: Boolean = false,
    configuration: C
): BasicDifferentialDrivetrain where M : MotorControllerGroup, M : MotorConfigurable<C> =
    BasicDifferentialDrivetrain(
        leftMotors = leftMotors.apply { configure(configuration) },
        rightMotors = rightMotors.apply { configure(configuration) },
        invertMotors = invertMotors
    )

/**
 * A simple implementation of a [DifferentialDrivetrain].
 *
 * @see DifferentialDrivetrain
 */
public open class BasicDifferentialDrivetrain(
    leftMotors: MotorControllerGroup,
    rightMotors: MotorControllerGroup,
    invertMotors: Boolean = false,
    protected val powerScale: Double = 1.0,
    protected val rotationScale: Double = 1.0,
) : SubsystemBase(), DifferentialDrivetrain {
    protected val differentialDrive: DifferentialDrive = DifferentialDrive(leftMotors, rightMotors)

    init {
        leftMotors.inverted = false
        rightMotors.inverted = true

        if (invertMotors) {
            leftMotors.inverted = !leftMotors.inverted
            rightMotors.inverted = !rightMotors.inverted
        }
    }

    override fun tankDrive(leftPower: Double, rightPower: Double) {
        differentialDrive.tankDrive(
            leftPower * powerScale,
            rightPower * powerScale,
            false
        )
    }

    override fun arcadeDrive(power: Double, rotation: Double) {
        differentialDrive.arcadeDrive(
            power * powerScale,
            rotation * rotationScale,
            false
        )
    }

    override fun curvatureDrive(power: Double, steering: Double) {
        differentialDrive.curvatureDrive(
            power * powerScale,
            steering * rotationScale,
            true
        )
    }

    override fun stop() {
        differentialDrive.stopMotor()
    }
}