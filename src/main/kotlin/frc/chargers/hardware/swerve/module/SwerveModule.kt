package frc.chargers.hardware.swerve.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.pid.AngularProfiledPIDController
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.swerve.control.TurnPID
import frc.chargers.hardware.swerve.control.VelocityPID
import frc.chargers.utils.Precision
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.swerve.direction
import org.littletonrobotics.junction.Logger


public class SwerveModule(
    private val io: ModuleIO,
    turnControl: TurnPID,
    velocityControl: VelocityPID,
    private val staticVoltageStall: Boolean = false
){

    private val staticStallVoltage = velocityControl.ff.kS


    private val inputs: ModuleIO.Inputs = ModuleIO.Inputs()

    public fun updateInputsAndLog(logName: String) {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs(logName,inputs)
    }



    private val velocityController = UnitSuperPIDController(
        velocityControl.pidConstants,
        {inputs.speed},
        -12.volts..12.volts,
        target = AngularVelocity(0.0),
        feedforward = velocityControl.ff,
        selfSustain = true
    )

    private val turnPrecision = when(turnControl){
        is TurnPID.Basic -> turnControl.precision
        is TurnPID.Profiled -> turnControl.precision
    }

    private val turnController: FeedbackController<Angle, Voltage> =
        when(turnControl){
            is TurnPID.Basic -> UnitSuperPIDController(
                turnControl.pidConstants,
                {inputs.direction % 360.degrees},
                -12.volts..12.volts,
                target = Angle(0.0),
                selfSustain = true
            )

            is TurnPID.Profiled -> AngularProfiledPIDController(
                turnControl.pidConstants,
                {inputs.direction % 360.degrees},
                -12.volts..12.volts,
                target = Angle(0.0),
                constraints = turnControl.constraints,
                feedforward = turnControl.turnFF,
                selfSustain = true
            )
        }


    public val currentDirection: Angle
        get() = inputs.direction

    public fun setDirection(direction: Angle) {
        turnController.target = direction
        // custom extension function
        if(turnPrecision is Precision.Within && turnController.error in turnPrecision.allowableError){
            io.setTurnVoltage(0.0.volts)
        }else{
            io.setTurnVoltage(turnController.calculateOutput())
        }
    }

    public fun setPower(power: Double) {
        io.setDriveVoltage(power * 12.volts)
    }

    public val currentVelocity: AngularVelocity
        get() = inputs.speed

    public fun setVelocity(velocity: AngularVelocity) {
        velocityController.target = velocity
        io.setDriveVoltage(velocityController.calculateOutput())
    }

    public val wheelPosition: Angle
        get() = inputs.distance

    public fun setDirectionalPower(
        power:Double,
        direction:Angle
    ){
        if (abs(direction - inputs.direction) > 90.degrees){
            setPower(-power)
            setDirection((direction + 180.degrees) % 360.degrees)
        }else{
            setPower(power)
            setDirection(direction % 360.degrees)
        }
    }

    public fun setDirectionalVelocity(
        angularVelocity:AngularVelocity,
        direction:Angle
    ){
        if (abs(direction - inputs.direction) > 90.degrees){
            setVelocity(-angularVelocity)
            setDirection((direction + 180.degrees) % 360.degrees)
        }else{
            setVelocity(angularVelocity)
            setDirection(direction % 360.degrees)
        }
    }


    public fun getModuleState(gearRatio:Double,wheelDiameter:Length): SwerveModuleState =
        SwerveModuleState(
            (currentVelocity * gearRatio * wheelDiameter / 2).inUnit(meters / seconds),
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

    public fun halt() {
        if(staticVoltageStall){
            io.setDriveVoltage(staticStallVoltage)
        }else{
            io.setDriveVoltage(0.0.volts)
        }
        io.setTurnVoltage(0.0.volts)
    }


}