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
import frc.chargers.hardware.subsystemutils.swervedrive.SecondOrderControlScheme
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.geometry.motion.AngularTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.rotation.asRotation2d
import org.littletonrobotics.junction.Logger

public class SwerveModule(
    private val name: String,
    public val io: ModuleIO,
    private val controlScheme: SwerveControl
){

    private val tuner = DashboardTuner()

    private val inputs: ModuleIO.Inputs = ModuleIO.Inputs().apply{
        // this input is not continuously updated nor used for anything else;
        // thus, it is only updated once.
        secondOrderKinematicsUtilized = controlScheme is SecondOrderControlScheme
    }

    /**
     * A function that standardizes all angles within the 0 to 360 degree range.
     */
    private fun Angle.standardize(): Angle = this.inputModulus(0.0.degrees..360.degrees)

    public fun updateAndProcessInputs() {
        // updates the data from the variables
        io.updateInputs(inputs)
        // the Inputs class implements the LoggableInputs interface,
        // with toLog and fromLog functions, implemented by the user.
        // toLog pushes variable data to the logger,
        // while fromLog is used in replay, and overrides user data based off of logs.
        // this function simply calls toLog and fromLog when nessecary on the data every loop.
        Logger.getInstance().processInputs(name,inputs)
        // periodically refreshes controllers to make sure they don't break.
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

        when (controlScheme) {
            // called when the control scheme uses BASIC PID control with second order kinematics
            is SwerveControl.PIDSecondOrder -> {
                turnController.target = direction.standardize()

                val extraTurnVoltage = controlScheme.turnFF.calculate(secondOrderTurnSpeed)
                // the "within" method is an extension function of Quantity<D: AnyDimension>
                if( (turnController.error).within(controlScheme.turnPrecision) ){
                    io.setTurnVoltage(extraTurnVoltage)
                }else{
                    io.setTurnVoltage(turnController.calculateOutput() + extraTurnVoltage)
                }

                // these properties of the inputs class are not used anywhere; just extra useful log data
                // (they are automatically pushed to the logger via AdvantageKit's processInputs function,
                // found in the updateAndProcessInputs function of this class).
                inputs.secondOrderTurnVoltage = extraTurnVoltage
                inputs.secondOrderTurnSpeed = secondOrderTurnSpeed
            }

            // called when the control scheme uses a PROFILED pid controller + second order kinematics
            is SwerveControl.ProfiledPIDSecondOrder -> {
                // cast is 100% safe; turnController can only become an AngularProfiledPIDcontroller
                // if the control scheme is profiled.
                (turnController as AngularProfiledPIDController).targetState = AngularTrapezoidProfile.State(
                    direction.standardize(),
                    secondOrderTurnSpeed
                )

                val extraTurnVoltage = controlScheme.turnFF.calculate(secondOrderTurnSpeed)
                if( (turnController.error).within(controlScheme.turnPrecision) ){
                    io.setTurnVoltage(extraTurnVoltage)
                }else{
                    // the turn speed is the TARGET STATE of the profiled PID controller;
                    // thus, no extra turn voltage is added.
                    io.setTurnVoltage(turnController.calculateOutput())
                }

                // these properties of the inputs class are not used anywhere; just extra useful log data
                // (they are automatically pushed to the logger via AdvantageKit's processInputs function,
                // found in the updateAndProcessInputs function of this class).
                inputs.secondOrderTurnVoltage = extraTurnVoltage
                inputs.secondOrderTurnSpeed = secondOrderTurnSpeed
            }

            // called when the control scheme uses first order kinematics.
            else -> {
                if (secondOrderTurnSpeed != AngularVelocity(0.0)){
                    println("WARNING: Second order turn speed is not being used. Something is wrong with the system.")
                }

                // if PID control is first order, simply take the output of the controller.
                turnController.target = direction.standardize()
                if( (turnController.error).within(controlScheme.turnPrecision) ){
                    io.setTurnVoltage(0.0.volts)
                }else{
                    io.setTurnVoltage(turnController.calculateOutput())
                }

            }
        }

        Logger.getInstance().apply{
            recordOutput("Drivetrain(Swerve)/$name/controllerErrorRad", turnController.error.inUnit(radians))
            recordOutput("Drivetrain(Swerve)/$name/controllerOutputVolts", turnController.calculateOutput().inUnit(volts))
        }


    }

    public fun setPower(power: Double) {
        io.setDriveVoltage(power * 12.volts)
    }

    public val currentVelocity: AngularVelocity
        get() = inputs.speed

    public val currentTurningVelocity: AngularVelocity
        get() = inputs.turnSpeed

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
        secondOrderTurnSpeed: AngularVelocity = AngularVelocity(0.0)
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
        setDirection(inputs.direction)
    }


}