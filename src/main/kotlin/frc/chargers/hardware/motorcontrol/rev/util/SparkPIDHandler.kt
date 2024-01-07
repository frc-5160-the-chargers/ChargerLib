package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.controls.pid.PIDConstants

/**
 * A utility class that manages closed loop control
 * for a CAN spark device.
 */
public class SparkPIDHandler(
    motor: com.revrobotics.CANSparkBase
) {

    private val innerController = motor.pidController
    private var currentPIDConstants = PIDConstants(0.0,0.0,0.0)
    private var currentFFConstants = AngularMotorFFConstants.None
    private var currentFF = Feedforward<AngularVelocityDimension, VoltageDimension>{ Voltage(0.0) }
    private var isCurrentlyWrapping = false

    private fun updateControllerConstants(newConstants: PIDConstants){
        if (currentPIDConstants != newConstants){
            innerController.setP(newConstants.kP,0)
            innerController.setI(newConstants.kI,0)
            innerController.setD(newConstants.kD,0)
            currentPIDConstants = newConstants
        }
    }

    public fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
    ) {
        updateControllerConstants(pidConstants)
        if (currentFFConstants != feedforwardConstants){
            currentFFConstants = feedforwardConstants
            currentFF = Feedforward(currentFFConstants)
        }
        innerController.setReference(
            target.siValue,
            com.revrobotics.CANSparkBase.ControlType.kVelocity,
            0,
            currentFF.calculate(target).inUnit(volts)
        )
    }

    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean,
        extraVoltage: Voltage
    ) {
        if (continuousWrap != isCurrentlyWrapping){
            if (continuousWrap){
                innerController.positionPIDWrappingEnabled = true
                innerController.positionPIDWrappingMinInput = -180.degrees.siValue
                innerController.positionPIDWrappingMaxInput = 180.degrees.siValue
            }else{
                innerController.positionPIDWrappingEnabled = false
            }
            isCurrentlyWrapping = continuousWrap
        }
        updateControllerConstants(pidConstants)
        innerController.setReference(
            target.siValue,
            com.revrobotics.CANSparkBase.ControlType.kPosition,
            0,
            extraVoltage.siValue
        )
    }

}