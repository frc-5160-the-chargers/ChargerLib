package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle

import frc.chargers.hardware.sensors.encoders.Encoder
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
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
                          public val invertTurnMotors: Boolean = false,
                          public val invertDriveMotors: Boolean = false,
                          vararg driveMotors: EncoderMotorController){
    public var multipliers: MutableList<Double> = mutableListOf()

    public var driveEncoder: Encoder = AverageEncoder(*driveMotors)



    // sets the multipliers of each motor inputted into the swerve module.
    public fun setMultipliers(vararg inputedMultipliers: Double){
        multipliers.clear()
        multipliers.addAll(inputedMultipliers.toList())
    }



    public var drivePower: Double = 0.0
    public val driveMotorList: List<EncoderMotorController> = driveMotors.toList()

    // note: I don't use unitSuperPID Controller cuz I don't know how to make it output a double.
    public val turnPID: SuperPIDController = SuperPIDController(
        pidConstants = turningPIDConstants,
        getInput = {turnEncoder.angularPosition.inUnit(Degrees)},
        target = 0.0
    )

    init{
        turnPID.basePID.enableContinuousInput(-180.0,180.0)
    }

    // gets the module state, using WPILib.
    // inUnit converts the angles and velocities into meters per second and degrees.
    // rotation2d is just used to format everything.
    public var moduleState: SwerveModuleState = SwerveModuleState()
        get() = SwerveModuleState(driveEncoder.angularVelocity.inUnit(Degrees/seconds) * wheelDiameter.inUnit(meters)/2,
            Rotation2d.fromDegrees(turnEncoder.angularPosition.inUnit(Degrees)))

    // note: setDirectionalPower should be called repeatedly
    // the superPID controller is called repeatedly here
    public fun setDirectionalPower(targetAngle: Angle,power: Double){
        var frontAngle: Angle = turnEncoder.angularPosition
        var backAngle: Angle = frontAngle + 180.degrees
        if (abs((backAngle-targetAngle).inUnit(Degrees)) > abs((frontAngle-targetAngle).inUnit(Degrees)) ){
            turnPID.target = (targetAngle-backAngle).inUnit(Degrees)
            drivePower = -power
        }else{
            turnPID.target = (targetAngle-frontAngle).inUnit(Degrees)
            drivePower = power
        }

        if (frontAngle != targetAngle || backAngle != targetAngle){
            turnMotor.set(turnPID.calculateOutput())
        }else{
            var counter = 0
            driveMotorList.forEach{i -> i.set(drivePower*multipliers[counter]);counter++}
        }
    }
}