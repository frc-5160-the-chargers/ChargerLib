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


public class SwerveModule(
    private val turnMotor: EncoderMotorController,
    private val turnEncoder: PositionEncoder,
    turnControl: TurnPID,
    private val driveMotor: EncoderMotorController,
    private val velocityControl: VelocityPID,
    private val staticVoltageStall: Boolean = false
): HolonomicModule {

    private val velocityController = UnitSuperPIDController(
        velocityControl.pidConstants,
        {driveMotor.encoder.angularVelocity},
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
                {turnEncoder.angularPosition},
                -12.volts..12.volts,
                target = Angle(0.0),
                selfSustain = true
            )

            is TurnPID.Profiled -> AngularProfiledPIDController(
                turnControl.pidConstants,
                {turnEncoder.angularPosition},
                -12.volts..12.volts,
                target = Angle(0.0),
                constraints = turnControl.constraints,
                feedforward = turnControl.turnFF,
                selfSustain = true
            )
        }


    override val currentDirection: Angle
        get() = turnEncoder.angularPosition

    override fun setDirection(direction: Angle) {
        turnController.target = direction
        // custom extension function
        if(turnPrecision is Precision.Within && turnController.error in turnPrecision.allowableError){
            turnMotor.set(0.0)
        }else{
            turnMotor.setVoltage(turnController.calculateOutput())
        }
    }

    override fun setPower(power: Double) {
        driveMotor.set(power)
    }

    override val currentVelocity: AngularVelocity
        get() = driveMotor.encoder.angularVelocity

    override fun setVelocity(velocity: AngularVelocity) {
        velocityController.target = velocity
        driveMotor.setVoltage(velocityController.calculateOutput())
    }

    override val wheelPosition: Angle
        get() = driveMotor.encoder.angularPosition

    override fun optimizeDirection(inputAngle: Angle): DirectionOptimizationData {
        val oldState = SwerveModuleState(1.0,inputAngle.asRotation2d())
        val newState = SwerveModuleState.optimize(oldState,turnEncoder.angularPosition.asRotation2d())
        return DirectionOptimizationData(
            // extension property
            newDirection = newState.direction,
            driveMultiplier = newState.speedMetersPerSecond
        )
    }

    override fun halt() {
        if(staticVoltageStall){
            driveMotor.setVoltage(velocityControl.ff.kS)
        }else{
            driveMotor.set(0.0)
        }
    }


}