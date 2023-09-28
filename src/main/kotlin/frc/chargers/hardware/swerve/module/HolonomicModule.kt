package frc.chargers.hardware.swerve.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.wpilibextensions.geometry.asRotation2d




public class DirectionOptimizationData(
    internal val newDirection: Angle,
    internal val driveMultiplier: Double
)


/**
 * Represents a module of a holonomic drivetrain.
 * It usually contains a turn motor and a drive motor combined, allowing for omnidirectional movement.
 *
 */
public interface HolonomicModule {


    public val currentDirection: Angle
    public fun setDirection(direction: Angle)

    public fun setPower(power: Double)

    public val currentVelocity: AngularVelocity
    public fun setVelocity(velocity: AngularVelocity)

    public val wheelPosition: Angle
    public fun optimizeDirection(inputAngle: Angle): DirectionOptimizationData




    public fun setDirectionalPower(
        power:Double,
        direction:Angle
    ){
        val optimizationData = optimizeDirection(direction)
        setPower(power * optimizationData.driveMultiplier)
        setDirection(optimizationData.newDirection)
    }

    public fun setDirectionalVelocity(
        angularVelocity:AngularVelocity,
        direction:Angle
    ){
        val optimizationData = optimizeDirection(direction)
        setVelocity(angularVelocity * optimizationData.driveMultiplier)
        setDirection(optimizationData.newDirection)
    }


    public fun getModuleState(gearRatio:Double,wheelDiameter:Length): SwerveModuleState =
        SwerveModuleState(
            (currentVelocity * gearRatio * wheelDiameter / 2).inUnit(meters/seconds),
            currentDirection.asRotation2d()
        )

    public fun getModulePosition(gearRatio:Double,wheelDiameter:Length): SwerveModulePosition =
        SwerveModulePosition(
            (wheelPosition * gearRatio * wheelDiameter / 2).inUnit(meters),
            currentDirection.asRotation2d()
        )

    public fun setDirectionalVelocity(velocity:Velocity,direction:Angle,gearRatio:Double,wheelDiameter:Length){
        setDirectionalVelocity(velocity/(gearRatio*wheelDiameter),direction)
    }

    public fun halt(){
        setPower(0.0)
    }
}


