package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Length
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.hardware.subsystemutils.swervedrive.module.lowlevel.ModuleIO

/**
 * Represents a swerve module within an
 * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 *
 * Extends ModuleIO to provide implementing classes
 * with low-level control, I.E setting voltage and getting encoder position.
 */
public interface SwerveModule: ModuleIO {
    public fun setDirectionalPower(
        power: Double,
        direction: Angle
    )

    public fun setDirectionalVelocity(
        angularVelocity: AngularVelocity,
        direction: Angle
    )

    public fun getModuleState(wheelRadius: Length): SwerveModuleState

    public fun getModulePosition(wheelRadius: Length): SwerveModulePosition

    public fun halt()
}