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
import org.littletonrobotics.junction.Logger


public class SwerveModule(
    public val io: ModuleIO,
    turnControl: TurnPID,
    velocityControl: VelocityPID,
    private val staticVoltageStall: Boolean = false
){
    private var logName = ""

    private val staticStallVoltage = velocityControl.ff.kS


    private val inputs: ModuleIO.Inputs = ModuleIO.Inputs()

    public fun updateInputsAndLog(logName: String) {
        io.updateInputs(inputs)
        this.logName = logName
        Logger.getInstance().processInputs(this.logName,inputs)
        velocityController.calculateOutput()
        turnController.calculateOutput()
    }

    private fun angleDeltaBetween(angleOne: Angle, angleTwo: Angle): Angle{
        // standardizes the angles into the 0-360 degree range
        val a1 = angleOne % 360.degrees
        val a2 = angleTwo % 360.degrees

        val result = abs(a1-a2)

        return if(360.degrees - result < result){
            360.degrees-result
        }else{
            result
        }
    }




    private val velocityController = UnitSuperPIDController(
        velocityControl.pidConstants,
        {inputs.speed},
        -12.volts..12.volts,
        target = AngularVelocity(0.0),
        feedforward = velocityControl.ff
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
                outputRange = -12.volts..12.volts,
                continuousInputRange = 0.degrees..360.degrees,
                target = Angle(0.0)
            )

            is TurnPID.Profiled -> AngularProfiledPIDController(
                turnControl.pidConstants,
                {inputs.direction % 360.degrees},
                outputRange = -12.volts..12.volts,
                continuousInputRange = 0.degrees..360.degrees,
                target = Angle(0.0),
                constraints = turnControl.constraints,
                feedforward = turnControl.turnFF
            )
        }


    public val currentDirection: Angle
        get() = inputs.direction % 360.degrees

    public fun setDirection(direction: Angle) {
        turnController.target = direction % 360.degrees
        if (logName != ""){
            Logger.getInstance().recordOutput("$logName/postOptimizeDesiredDirection", turnController.target.inUnit(degrees))
        }
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
        if (logName != ""){
            Logger.getInstance().recordOutput("$logName/initialDesiredDirection", direction.inUnit(degrees))
            Logger.getInstance().recordOutput("$logName/angleDelta", abs(direction - inputs.direction).inUnit(degrees))
        }
        if (angleDeltaBetween(direction,inputs.direction) > 90.0.degrees){
            setPower(-power)
            setDirection(direction + 180.degrees)
        }else{
            setPower(power)
            setDirection(direction)
        }
    }

    public fun setDirectionalVelocity(
        angularVelocity:AngularVelocity,
        direction:Angle
    ){
        if (logName != ""){
            Logger.getInstance().recordOutput("$logName/initialDesiredDirection", direction.inUnit(degrees))
            Logger.getInstance().recordOutput("$logName/angleDelta", angleDeltaBetween(direction,inputs.direction).inUnit(degrees))
        }
        if (angleDeltaBetween(direction,inputs.direction) > 90.0.degrees){
            setVelocity(-angularVelocity)
            setDirection(direction + 180.degrees)
        }else{
            setVelocity(angularVelocity)
            setDirection(direction)
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