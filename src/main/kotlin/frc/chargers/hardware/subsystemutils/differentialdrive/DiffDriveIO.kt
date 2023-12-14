package frc.chargers.hardware.subsystemutils.differentialdrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import frc.chargers.wpilibextensions.motorcontrol.setVoltage



public interface DiffDriveIO {
    public val leftWheelTravel: Angle
    public val rightWheelTravel: Angle

    public val leftVelocity: AngularVelocity
    public val rightVelocity: AngularVelocity

    public val leftVoltage: Voltage
    public val rightVoltage: Voltage

    public fun setVoltages(left: Voltage, right: Voltage)

    public var inverted: Boolean
}