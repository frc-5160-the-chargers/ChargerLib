package frc.chargers.hardware.swerve.module

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.pid.AngularProfiledPIDController
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.swerve.control.ProfiledPIDControlScheme
import frc.chargers.hardware.swerve.control.SwerveControl
import frc.chargers.utils.Precision
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.geometry.asRotation2d
import org.littletonrobotics.junction.Logger

public class SwerveModule(
    public val io: ModuleIO,
    private val controlScheme: SwerveControl
){

    private val staticStallVoltage = controlScheme.driveFF.kS


    private val inputs: ModuleIO.Inputs = ModuleIO.Inputs()

    /**
     * A function that standardizes all angles within the 0 to 360 degree range.
     */
    private fun Angle.standardize(): Angle = if (this < Angle(0.0)){
        (this % 360.degrees) + 360.degrees
    }else{
        this % 360.degrees
    }

    public fun updateAndProcessInputs(logName: String) {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs(logName,inputs)
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




    public val velocityController: UnitSuperPIDController<AngularVelocityDimension, VoltageDimension> = UnitSuperPIDController(
        controlScheme.drivePIDConstants,
        {inputs.speed},
        -12.volts..12.volts,
        target = AngularVelocity(0.0),
        feedforward = controlScheme.driveFF
    )



    private val turnController: FeedbackController<Angle, Voltage> =
        when(controlScheme){

            is ProfiledPIDControlScheme -> AngularProfiledPIDController(
                controlScheme.turnPIDConstants,
                {inputs.direction.standardize()},
                outputRange = -12.volts..12.volts,
                continuousInputRange = 0.degrees..360.degrees,
                target = Angle(0.0),
                constraints = controlScheme.turnConstraints,
                feedforward = controlScheme.turnFF
            )

            else -> UnitSuperPIDController(
                controlScheme.turnPIDConstants,
                {inputs.direction.standardize()},
                outputRange = -12.volts..12.volts,
                continuousInputRange = 0.degrees..360.degrees,
                target = Angle(0.0)
            )
        }


    public val currentDirection: Angle
        get() = inputs.direction.standardize()

    public fun setDirection(direction: Angle, extraTurnVoltage: Voltage = Voltage(0.0)) {
        turnController.target = direction.standardize()
        // custom extension function
        if(controlScheme.turnPrecision is Precision.Within && turnController.error in controlScheme.turnPrecision.allowableError){
            io.setTurnVoltage(0.0.volts)
        }else{
            io.setTurnVoltage(turnController.calculateOutput() + extraTurnVoltage)
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
        extraTurnVoltage: Voltage = Voltage(0.0)
    ){
        if (angleDeltaBetween(direction,inputs.direction) > 90.0.degrees){
            setDirection(direction + 180.degrees,extraTurnVoltage)
            setPower(-power * cos(turnController.error))
        }else{
            setDirection(direction,extraTurnVoltage)
            setPower(power * cos(turnController.error))
        }
    }

    public fun setDirectionalVelocity(
        angularVelocity:AngularVelocity,
        direction:Angle,
        extraTurnVoltage: Voltage = Voltage(0.0)
    ){
        if (angleDeltaBetween(direction,inputs.direction) > 90.0.degrees){
            setDirection(direction + 180.degrees,extraTurnVoltage)
            setVelocity(-angularVelocity * cos(turnController.error))

        }else{
            setDirection(direction,extraTurnVoltage)
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
            io.setDriveVoltage(staticStallVoltage)
        }else{
            io.setDriveVoltage(0.0.volts)
        }
        io.setTurnVoltage(0.0.volts)
    }


}