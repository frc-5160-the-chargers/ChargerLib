package frc.chargers.hardware.subsystems.differentialdrive.lowlevel

import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import com.batterystaple.kmeasure.quantities.*
import frc.chargers.hardware.motorcontrol.setVoltage

public class DiffDriveIOReal(
    logInputs: LoggableInputsProvider,
    private val leftMotors: NonConfigurableEncoderMotorControllerGroup,
    private val rightMotors: NonConfigurableEncoderMotorControllerGroup
): DiffDriveIO {

    init {
        leftMotors.inverted = false
        rightMotors.inverted = true
    }

    private var leftAppliedVoltage = Voltage(0.0)
    private var rightAppliedVoltage = Voltage(0.0)

    override val leftWheelTravel: Angle by logInputs.quantity{leftMotors.encoder.angularPosition}
    override val rightWheelTravel: Angle by logInputs.quantity{rightMotors.encoder.angularPosition}

    override val leftVelocity: AngularVelocity by logInputs.quantity{leftMotors.encoder.angularVelocity}
    override val rightVelocity: AngularVelocity by logInputs.quantity{rightMotors.encoder.angularVelocity}

    override val leftVoltage: Voltage by logInputs.quantity{leftAppliedVoltage}
    override val rightVoltage: Voltage by logInputs.quantity {rightAppliedVoltage}

    override fun setVoltages(left: Voltage, right: Voltage) {
        leftAppliedVoltage = left
        rightAppliedVoltage = right
        // uses custom extension functions; see wpilibextensions
        leftMotors.setVoltage(left)
        rightMotors.setVoltage(right)
    }


    override var inverted: Boolean = false
        set(invertMotors){
            if (invertMotors) {
                leftMotors.inverted = true
                rightMotors.inverted = false
            }else{
                leftMotors.inverted = false
                rightMotors.inverted = true
            }
            field = invertMotors
        }
}