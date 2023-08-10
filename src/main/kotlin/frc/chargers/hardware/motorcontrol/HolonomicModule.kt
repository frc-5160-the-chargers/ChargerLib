package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.wpilibextensions.geometry.asAngle

/**
 * Represents a module of a holonomic drivetrain,
 * where the turn and drive motors can be configured.
 *
 * @see NonConfigurableHolonomicModule
 */
public interface HolonomicModule<TMC: MotorConfiguration, DMC: MotorConfiguration>: NonConfigurableHolonomicModule{
    public fun configureDriveMotor(configuration: DMC)
    public fun configureTurnMotor(configuration: TMC)
}



/**
 * Represents a module of a holonomic drivetrain.
 * It usually contains a turn motor and a drive motor combined, allowing for omnidirectional movement.
 *
 */
public interface NonConfigurableHolonomicModule {

    public val distanceMeasurementEncoder: Encoder
    public fun setDirectionalPower(power:Double,direction:Angle)

    public fun setDirectionalVelocity(angularVelocity:AngularVelocity,direction:Angle)

    public fun getModuleState(gearRatio:Double,wheelDiameter:Length): SwerveModuleState

    public fun getModulePosition(gearRatio:Double,wheelDiameter:Length): SwerveModulePosition

    public fun setDirectionalVelocity(velocity:Velocity,direction:Angle,gearRatio:Double,wheelDiameter:Length){
        setDirectionalVelocity(velocity/(gearRatio*wheelDiameter),direction)
    }
    public fun setDesiredState(moduleState:SwerveModuleState,gearRatio:Double,wheelDiameter:Length){
        setDirectionalVelocity(
            moduleState.speedMetersPerSecond.ofUnit(meters/seconds),
            moduleState.angle.asAngle(),
            gearRatio,
            wheelDiameter
        )
    }

    public fun halt()
}


