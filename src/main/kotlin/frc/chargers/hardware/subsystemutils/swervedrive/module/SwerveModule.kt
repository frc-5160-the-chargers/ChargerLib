package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.pid.AngularProfiledPIDController
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.subsystemutils.swervedrive.ProfiledPIDControlScheme
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.utils.Precision
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.asRotation2d
import org.littletonrobotics.junction.Logger

public class SwerveModule(
    private val name: String,
    public val io: ModuleIO,
    private val controlScheme: SwerveControl
){

    private val tuner = DashboardTuner()

    private val inputs: ModuleIO.Inputs = ModuleIO.Inputs()

    /**
     * A function that standardizes all angles within the 0 to 360 degree range.
     */
    private fun Angle.standardize(): Angle = if (this < Angle(0.0)){
        (this % 360.degrees) + 360.degrees
    }else{
        this % 360.degrees
    }

    public fun updateAndProcessInputs() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs(name,inputs)
        velocityController.calculateOutput()
        turnController.calculateOutput()
    }

    /**
     * A function used to calculate the smallest angle delta between 2 angles.
     */
    private fun angleDeltaBetween(angleOne: Angle, angleTwo: Angle): Angle{
        val a1 = angleOne.standardize()
        val a2 = angleTwo.standardize()

        val result = abs(a1-a2)

        return if(360.degrees - result < result){
            360.degrees-result
        }else{
            result
        }
    }

    private val turnPIDConstants by tuner.pidConstants(
        controlScheme.turnPIDConstants,
        "$name/Turning PID Constants"
    )

    private val drivePIDConstants by tuner.pidConstants(
        controlScheme.drivePIDConstants,
        "$name/Driving PID Constants"
    )




    public val velocityController: UnitSuperPIDController<AngularVelocityDimension, VoltageDimension>
        by tuner.refreshWhenTuned{
            UnitSuperPIDController(
                drivePIDConstants,
                {inputs.speed},
                -12.volts..12.volts,
                target = AngularVelocity(0.0),
                feedforward = controlScheme.driveFF
            )
        }



    private val turnController: FeedbackController<Angle, Voltage>
        by tuner.refreshWhenTuned{
            when(controlScheme){
                is ProfiledPIDControlScheme -> AngularProfiledPIDController(
                    turnPIDConstants,
                    getInput = {inputs.direction.standardize()},
                    outputRange = -12.volts..12.volts,
                    continuousInputRange = 0.degrees..360.degrees,
                    target = Angle(0.0),
                    constraints = controlScheme.turnConstraints,
                    feedforward = controlScheme.turnFF
                )

                else -> UnitSuperPIDController(
                    turnPIDConstants,
                    getInput = {inputs.direction.standardize()},
                    outputRange = -12.volts..12.volts,
                    continuousInputRange = 0.degrees..360.degrees,
                    target = Angle(0.0)
                )
            }
        }



    public val currentDirection: Angle
        get() = inputs.direction.standardize()



    // Note: turnSpeed will only be set if the control scheme includes second order kinematics functionality.
    public fun setDirection(direction: Angle, secondOrderTurnSpeed: AngularVelocity = AngularVelocity(0.0)){

        if(controlScheme.turnPrecision is Precision.Within && turnController.error in controlScheme.turnPrecision.allowableError){
            io.setTurnVoltage(0.0.volts)
            return
        }


        when (controlScheme) {
            is SwerveControl.PIDSecondOrder -> {
                turnController.target = direction.standardize()
                io.setTurnVoltage(
                    turnController.calculateOutput() + controlScheme.turnFF.calculate(secondOrderTurnSpeed)
                )
                Logger.getInstance().recordOutput("$name/SecondOrderTurnSpeedRadPerSec", secondOrderTurnSpeed.inUnit(radians/seconds))
            }

            is SwerveControl.ProfiledPIDSecondOrder -> {
                // cast is 100% safe; turnController can only become an AngularProfiledPIDcontroller
                // if the control scheme is profiled.
                (turnController as AngularProfiledPIDController).targetState = AngularTrapezoidProfile.State(
                    direction.standardize(),
                    secondOrderTurnSpeed
                )

                io.setTurnVoltage(turnController.calculateOutput())

                Logger.getInstance().recordOutput("$name/SecondOrderTurnSpeedRadPerSec", secondOrderTurnSpeed.inUnit(radians/seconds))
            }

            else -> {
                if (secondOrderTurnSpeed != AngularVelocity(0.0)){
                    println("WARNING: Second order turn speed is not being used. Something is wrong with the system.")
                }else{
                    println("First order control currently being utilized")
                }

                // if PID control is first order, simply take the output of the controller.
                turnController.target = direction.standardize()
                io.setTurnVoltage(turnController.calculateOutput())
            }
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
        direction:Angle,
        secondOrderTurnSpeed: AngularVelocity = AngularVelocity(0.0)
    ){
        if (angleDeltaBetween(direction,inputs.direction) > 90.0.degrees){
            setDirection(direction + 180.degrees,secondOrderTurnSpeed)
            setPower(-power * cos(turnController.error))
        }else{
            setDirection(direction,secondOrderTurnSpeed)
            setPower(power * cos(turnController.error))
        }
    }

    public fun setDirectionalVelocity(
        angularVelocity:AngularVelocity,
        direction:Angle,
        secondOrderTurnSpeed: AngularVelocity
    ){
        if (angleDeltaBetween(direction,inputs.direction) > 90.0.degrees){
            setDirection(direction + 180.degrees,secondOrderTurnSpeed)
            setVelocity(-angularVelocity * cos(turnController.error))

        }else{
            setDirection(direction,secondOrderTurnSpeed)
            setVelocity(angularVelocity * cos(turnController.error))
        }
    }


    public fun getModuleState(wheelRadius: Length): SwerveModuleState =
        SwerveModuleState(
            currentVelocity.inUnit(radians / seconds) * wheelRadius.inUnit(meters),
            currentDirection.asRotation2d()
        )

    public fun getModulePosition(wheelRadius: Length): SwerveModulePosition =
        SwerveModulePosition(
            wheelPosition.inUnit(radians) * wheelRadius.inUnit(meters),
            currentDirection.asRotation2d()
        )


    public fun halt() {
        if(controlScheme.staticVoltageStall){
            io.setDriveVoltage(controlScheme.driveFF.kS)
        }else{
            io.setDriveVoltage(0.0.volts)
        }
        io.setTurnVoltage(0.0.volts)
    }


}