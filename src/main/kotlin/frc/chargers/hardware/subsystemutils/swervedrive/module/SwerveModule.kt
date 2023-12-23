package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.pid.AngularProfiledPIDController
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.geometry.rotation.asRotation2d
import org.littletonrobotics.junction.Logger.recordOutput

/**
 * Represents a single swerve module within an
 * [frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain].
 */
public class SwerveModule(
    lowLevel: ModuleIO,
    private val controlScheme: SwerveControl
): ModuleIO by lowLevel{
    private val tuner = DashboardTuner()

    /**
     * A function that standardizes all angles within the 0 to 360 degree range.
     */
    private fun Angle.standardize(): Angle = this.inputModulus(0.0.degrees..360.degrees)

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
        controlScheme.anglePID,
        "$logTab/Turning PID Constants"
    )

    private val drivePIDConstants by tuner.pidConstants(
        controlScheme.velocityPID,
        "$logTab/Driving PID Constants"
    )

    private val velocityController by tuner.refreshWhenTuned{
        UnitSuperPIDController(
            drivePIDConstants,
            {speed},
            -12.volts..12.volts,
            target = AngularVelocity(0.0),
            feedforward = controlScheme.velocityFF,
            selfSustain = true
        )
    }



    private val turnController: FeedbackController<Angle, Voltage>
        by tuner.refreshWhenTuned{
            when(controlScheme){
                is SwerveControl.ProfiledPID -> AngularProfiledPIDController(
                    turnPIDConstants,
                    getInput = {direction.standardize()},
                    outputRange = -12.volts..12.volts,
                    continuousInputRange = 0.degrees..360.degrees,
                    target = Angle(0.0),
                    constraints = controlScheme.angleTargetConstraints,
                    feedforward = controlScheme.angleTargetFF,
                    selfSustain = true
                )

                else -> UnitSuperPIDController(
                    turnPIDConstants,
                    getInput = {direction.standardize()},
                    outputRange = -12.volts..12.volts,
                    continuousInputRange = 0.degrees..360.degrees,
                    target = Angle(0.0),
                    selfSustain = true
                )
            }
        }


    // Note: turnSpeed will only be set if the control scheme includes second order kinematics functionality.
    public fun setDirection(direction: Angle){
        turnController.target = direction.standardize()
        // turnVoltage is a setter variable of ModuleIO
        turnVoltage = if( (turnController.error).within(controlScheme.modulePrecision) ){
            0.0.volts
        }else{
            turnController.calculateOutput()
        }
        recordOutput("$logTab/controllerErrorRad", turnController.error.inUnit(radians))
        recordOutput("$logTab/controllerOutputVolts", turnController.calculateOutput().inUnit(volts))
    }

    public fun setVelocity(velocity: AngularVelocity) {
        velocityController.target = velocity
        // driveVoltage is a setter variable of ModuleIO
        driveVoltage = velocityController.calculateOutput()
    }

    public fun setPower(power: Double) {
        driveVoltage = power * 12.volts
    }

    public fun setDirectionalPower(
        power: Double,
        direction: Angle
    ){
        if (angleDeltaBetween(direction,direction) > 90.0.degrees){
            setDirection(direction + 180.degrees)
            setPower(-power * cos(turnController.error))
        }else{
            setDirection(direction)
            setPower(power * cos(turnController.error))
        }
    }

    public fun setDirectionalVelocity(
        angularVelocity: AngularVelocity,
        direction: Angle
    ){
        if (angleDeltaBetween(direction,direction) > 90.0.degrees){
            setDirection(direction + 180.degrees)
            setVelocity(-angularVelocity * cos(turnController.error))
        }else{
            setDirection(direction)
            setVelocity(angularVelocity * cos(turnController.error))
        }
    }

    public fun getModuleState(wheelRadius: Length): SwerveModuleState =
        SwerveModuleState(
            speed.inUnit(radians / seconds) * wheelRadius.inUnit(meters),
            direction.asRotation2d()
        )

    public fun getModulePosition(wheelRadius: Length): SwerveModulePosition =
        SwerveModulePosition(
            wheelTravel.inUnit(radians) * wheelRadius.inUnit(meters),
            direction.asRotation2d()
        )

    public fun halt() {
        driveVoltage = 0.0.volts
        setDirection(direction)
    }

}