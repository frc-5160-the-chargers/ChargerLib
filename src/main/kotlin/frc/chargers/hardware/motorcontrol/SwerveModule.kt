package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle

import frc.chargers.hardware.sensors.encoders.Encoder
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.pid.FeedForward

import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import kotlin.math.abs

// note: either swerveModule.PIDOutput or swerveModule.setDirectionalPower must be called repeatedly
public class SwerveModule(public val turnMotor: EncoderMotorController,
                          public val turnEncoder: Encoder,
                          public val turningPIDConstants: PIDConstants,
                          public val turnFeedForward: FeedForward = FeedForward{_,_ -> 0.0},
                          private val wheelDiameter: Length,
                          public val driveMotor: EncoderMotorController,
                          public val turnMotorMultiplier: Double = 1.0,
                          public val driveMotorMultiplier: Double = 1.0){

    public var driveEncoder: Encoder = driveMotor.encoder





    public var drivePower: Double = 0.0


    // note: I don't use unitSuperPID Controller cuz I don't know how to make it output a double.
    public val turnPID: SuperPIDController = SuperPIDController(
        pidConstants = turningPIDConstants,
        getInput = {turnEncoder.angularPosition.inUnit(degrees)},
        target = 0.0
    )

    init{
        turnPID.basePID.enableContinuousInput(-180.0,180.0)
    }

    // gets the module state, using WPILib.
    // inUnit converts the angles and velocities into meters per second and degrees.
    // rotation2d is just used to format everything.
    public val moduleState: SwerveModuleState
        get() = SwerveModuleState(driveEncoder.angularVelocity.inUnit(degrees/seconds) * wheelDiameter.inUnit(meters)/2,
            Rotation2d.fromDegrees(turnEncoder.angularPosition.inUnit(degrees)))





    // note: setDirectionalPower should be called repeatedly
    // the superPID controller is called repeatedly here
    public fun setDirectionalPower(targetAngle: Angle,power: Double){
        var frontAngle: Angle = turnEncoder.angularPosition
        var backAngle: Angle = frontAngle + 180.degrees
        if (abs((backAngle-targetAngle).inUnit(degrees)) > abs((frontAngle-targetAngle).inUnit(degrees)) ){
            turnPID.target = (targetAngle-backAngle).inUnit(degrees)
            drivePower = -power*driveMotorMultiplier
        }else{
            turnPID.target = (targetAngle-frontAngle).inUnit(degrees)
            drivePower = power*driveMotorMultiplier
        }

        if (frontAngle != targetAngle || backAngle != targetAngle){
            turnMotor.set(turnPID.calculateOutput())
        }else{
            var counter = 0
            driveMotor.set(drivePower)
        }
    }

    public fun halt(){
        driveMotor.set(0.0)
    }
}