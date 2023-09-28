package frc.chargers.hardware.swerve.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.pid.AngularProfiledPIDController
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.swerve.control.TurnPID
import frc.chargers.hardware.swerve.control.VelocityPID
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.kinematics.swerve.direction
import frc.chargers.wpilibextensions.motorcontrol.setVoltage
import org.littletonrobotics.junction.Logger


public class SwerveModule(
    private val io: ModuleIO,
    turnControl: TurnPID,
    velocityControl: VelocityPID,
    private val staticVoltageStall: Boolean = false
): HolonomicModule {

    private val staticStallVoltage = velocityControl.ff.kS


    private val inputs: ModuleIO.Inputs = ModuleIO.Inputs()

    override fun updateInputsAndLog(logName: String) {
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
                {inputs.direction},
                -12.volts..12.volts,
                target = Angle(0.0),
                selfSustain = true
            )

            is TurnPID.Profiled -> AngularProfiledPIDController(
                turnControl.pidConstants,
                {inputs.direction},
                -12.volts..12.volts,
                target = Angle(0.0),
                constraints = turnControl.constraints,
                feedforward = turnControl.turnFF,
                selfSustain = true
            )
        }


    override val currentDirection: Angle
        get() = inputs.direction

    override fun setDirection(direction: Angle) {
        turnController.target = direction
        // custom extension function
        if(turnPrecision is Precision.Within && turnController.error in turnPrecision.allowableError){
            io.setTurnVoltage(0.0.volts)
        }else{
            io.setTurnVoltage(turnController.calculateOutput())
        }
    }

    override fun setPower(power: Double) {
        io.setDriveVoltage(power * 12.volts)
    }

    override val currentVelocity: AngularVelocity
        get() = inputs.speed

    override fun setVelocity(velocity: AngularVelocity) {
        velocityController.target = velocity
        io.setDriveVoltage(velocityController.calculateOutput())
    }

    override val wheelPosition: Angle
        get() = inputs.distance

    override fun optimizeDirection(inputAngle: Angle): DirectionOptimizationData {
        val oldState = SwerveModuleState(1.0,inputAngle.asRotation2d())
        val newState = SwerveModuleState.optimize(oldState,inputs.direction.asRotation2d())
        return DirectionOptimizationData(
            // extension property
            newDirection = newState.direction,
            driveMultiplier = newState.speedMetersPerSecond
        )
    }

    override fun halt() {
        if(staticVoltageStall){
            io.setDriveVoltage(staticStallVoltage)
        }else{
            io.setDriveVoltage(0.0.volts)
        }
        io.setTurnVoltage(0.0.volts)
    }


}