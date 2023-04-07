package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.LengthDimension
import com.batterystaple.kmeasure.dimensions.VelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.encoders.Encoder
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.controls.pid.*
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import kotlin.math.abs


/**
 * A convenience function to create a [SwerveModule]
 * using Talon FX Motor controllers.
 */
public fun TalonFXSwerveModule(
    turnMotor: ChargerTalonFX,
    turnEncoder: Encoder,
    turnPIDConstants: PIDConstants = PIDConstants(0.1,0.0,0.0),
    turnFeedForward: UnitFeedForward<AngleDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    driveMotor: ChargerTalonFX,
    velocityPIDConstants: PIDConstants = PIDConstants(0.1,0.0,0.0),
    velocityFeedForward: UnitFeedForward<VelocityDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    turnMotorMultiplier: Double = 1.0,
    driveMotorMultiplier: Double = 1.0,
    configuration: TalonFXConfiguration.() -> Unit,
): SwerveModule = SwerveModule(
    turnMotor.apply{configure(TalonFXConfiguration().apply(configuration))},
    turnEncoder,
    turnPIDConstants,
    turnFeedForward,
    driveMotor.apply{configure(TalonFXConfiguration().apply(configuration))},
    velocityPIDConstants,
    velocityFeedForward,
    turnMotorMultiplier,
    driveMotorMultiplier)

/**
 * A convenience function to create a [SwerveModule]
 * with spark max motor controllers.
 */
public fun SparkMaxSwerveModule(
    turnMotor: ChargerCANSparkMax,
    turnEncoder: Encoder,
    turnPIDConstants: PIDConstants = PIDConstants(0.1,0.0,0.0),
    turnFeedForward: UnitFeedForward<AngleDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    driveMotor: ChargerCANSparkMax,
    velocityPIDConstants: PIDConstants = PIDConstants(0.1,0.0,0.0),
    velocityFeedForward: UnitFeedForward<VelocityDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    turnMotorMultiplier: Double = 1.0,
    driveMotorMultiplier: Double = 1.0,
    configuration: SparkMaxConfiguration.() -> Unit,
): SwerveModule = SwerveModule(
    turnMotor.apply{configure(SparkMaxConfiguration().apply(configuration))},
    turnEncoder,
    turnPIDConstants,
    turnFeedForward,
    driveMotor.apply{configure(SparkMaxConfiguration().apply(configuration))},
    velocityPIDConstants,
    velocityFeedForward,
    turnMotorMultiplier,
    driveMotorMultiplier)




// IMPORTANT NOTE:
// To use this properly, you must do .apply{driveWheelTravelPerMotorRadin = something}
public open class SwerveModule(public val turnMotor: EncoderMotorController,
                          public val turnEncoder: Encoder,
                          public var turnPIDConstants: PIDConstants = PIDConstants(0.1,0.0,0.0),
                          public var turnFeedForward: UnitFeedForward<AngleDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
                          public val driveMotor: EncoderMotorController,
                          public var velocityPIDConstants: PIDConstants = PIDConstants(0.1,0.0,0.0),
                          public var velocityFeedForward: UnitFeedForward<VelocityDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
                          public val turnMotorMultiplier: Double = 1.0,
                          public val driveMotorMultiplier: Double = 1.0){

    public var driveEncoder: Encoder = driveMotor.encoder

    public var driveWheelTravelPerMotorRadian: Quantity<LengthDimension> = 0.0.meters


    public var drivePower: Double = 0.0

    // takes an input in degrees, and gives an output in volts.
    public val turnPID: UnitSuperPIDController<AngleDimension,VoltageDimension> = UnitSuperPIDController(
        pidConstants = turnPIDConstants,
        getInput = {driveEncoder.angularPosition},
        target = 0.0.degrees,
        feedForward = turnFeedForward
    )

    // takes an input in velocity, and gives an output in volts.
    public val velocityPID: UnitSuperPIDController<VelocityDimension,VoltageDimension> = UnitSuperPIDController(
        pidConstants = velocityPIDConstants,
        getInput = {driveEncoder.angularVelocity * driveWheelTravelPerMotorRadian},
        target = 0.0.meters/0.0.seconds,
        feedForward = velocityFeedForward
    )



    init{
        turnPID.enableContinuousInput(-180.0,180.0)
    }

    // gets the module state, using WPILib.
    // inUnit converts the angles and velocities into meters per second and degrees.
    // rotation2d is just used to format everything.
    public val moduleState: SwerveModuleState
        get() = SwerveModuleState((driveEncoder.angularVelocity * driveWheelTravelPerMotorRadian).inUnit(meters/seconds),
            Rotation2d.fromDegrees(turnEncoder.angularPosition.inUnit(degrees)))

    public fun setDesiredModuleState(rawState: SwerveModuleState){
        var optimizedState: SwerveModuleState = SwerveModuleState.optimize(rawState,
            Rotation2d(turnEncoder.angularPosition.inUnit(radians)))

        // this does look quite funny lol.
        // to explain, optimizedState.angle returns a rotation2d.
        // then, you can call .getRadians()(which converts to .radians as a getter in kotlin)
        // then, .radians converts this number to the radians Angle(from kmeasure)
        turnPID.target = optimizedState.angle.radians.radians

        velocityPID.target = optimizedState.speedMetersPerSecond.meters/1.seconds

        turnMotor.setVoltage(turnPID.calculateOutput().inUnit(volts))
        driveMotor.setVoltage(velocityPID.calculateOutput().inUnit(volts))
    }






    public fun setDirectionalPower(targetAngle: Angle,power: Double){
        var frontAngle: Angle = turnEncoder.angularPosition
        var backAngle: Angle = frontAngle + 180.degrees
        if (abs((backAngle-targetAngle).inUnit(degrees)) > abs((frontAngle-targetAngle).inUnit(degrees)) ){
            turnPID.target = targetAngle-backAngle
            drivePower = -power*driveMotorMultiplier
        }else{
            turnPID.target = targetAngle-frontAngle
            drivePower = power*driveMotorMultiplier
        }

        if (frontAngle != targetAngle || backAngle != targetAngle){
            turnMotor.setVoltage(turnPID.calculateOutput().inUnit(volts))
        }else{
            driveMotor.set(drivePower)
        }
    }



    public fun halt(){
        driveMotor.set(0.0)
    }
}