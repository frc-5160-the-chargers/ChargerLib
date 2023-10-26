package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import frc.chargers.wpilibextensions.motorcontrol.setVoltage

public class DifferentialDriveIOReal(
    private val leftMotors: NonConfigurableEncoderMotorControllerGroup,
    private val rightMotors: NonConfigurableEncoderMotorControllerGroup
): DifferentialDriveIO{

    init {
        leftMotors.inverted = false
        rightMotors.inverted = true

    }
    override fun setVoltages(left: Voltage, right: Voltage) {
        // uses custom extension functions; see wpilibextensions
        leftMotors.setVoltage(left)
        rightMotors.setVoltage(right)
    }

    override fun updateInputs(inputs: DifferentialDriveIO.Inputs){
        inputs.apply{
            leftAngularPosition = leftMotors.encoder.angularPosition
            rightAngularPosition = rightMotors.encoder.angularPosition

            leftAngularVelocity = leftMotors.encoder.angularVelocity
            rightAngularVelocity = rightMotors.encoder.angularVelocity
        }
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

public class DifferentialDriveIOSim(
    motors: DifferentialDrivetrainSim.KitbotMotor,
    private val loopPeriod: Time = 20.milli.seconds
): DifferentialDriveIO{
    private val sim: DifferentialDrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
        motors,
        DifferentialDrivetrainSim.KitbotGearing.k10p71,
        DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
        null
    )

    private var leftAppliedVoltage = Voltage(0.0)
    private var rightAppliedVoltage = Voltage(0.0)

    // gearRatio * wheelDiameter for simulator
    private val wheelTravelPerMotorRadian = 10.71 * 6.inches
    override fun setVoltages(left: Voltage, right: Voltage) {
        if (inverted){
            leftAppliedVoltage = left.coerceIn(-12.volts,12.volts)
            rightAppliedVoltage = right.coerceIn(-12.volts,12.volts)
        }else{
            leftAppliedVoltage = -left.coerceIn(-12.volts,12.volts)
            rightAppliedVoltage = -right.coerceIn(-12.volts,12.volts)
        }
        sim.setInputs(
            leftAppliedVoltage.inUnit(volts),
            rightAppliedVoltage.inUnit(volts)
        )
    }

    override fun updateInputs(inputs: DifferentialDriveIO.Inputs) {
        sim.update(loopPeriod.inUnit(seconds))
        inputs.apply{
            leftAngularPosition =
                sim.leftPositionMeters.ofUnit(meters) / wheelTravelPerMotorRadian
            rightAngularPosition =
                sim.rightPositionMeters.ofUnit(meters) / wheelTravelPerMotorRadian
            leftAngularVelocity =
                sim.leftVelocityMetersPerSecond.ofUnit(meters / seconds) / wheelTravelPerMotorRadian
            rightAngularVelocity =
                sim.rightVelocityMetersPerSecond.ofUnit(meters / seconds) / wheelTravelPerMotorRadian
        }
    }

    override var inverted: Boolean = false

}


public interface DifferentialDriveIO{
    public class Inputs: ChargerLoggableInputs(){
        public var leftAngularPosition: Angle by loggedQuantity(
            logUnit = degrees,
            "leftPositionDeg"
        )
        public var rightAngularPosition: Angle by loggedQuantity(
            logUnit = degrees,
            "rightPositionDeg"
        )

        public var leftAngularVelocity: AngularVelocity by loggedQuantity(
            logUnit = degrees / seconds,
            "leftVelocityDegPerSecond"
        )
        public var rightAngularVelocity: AngularVelocity by loggedQuantity(
            logUnit = degrees / seconds,
            "rightVelocityDegPerSecond"
        )

        public var leftVoltage: Voltage by loggedQuantity(
            logUnit = volts,
            "leftVoltage"
        )

        public var rightVoltage: Voltage by loggedQuantity(
            logUnit = volts,
            "rightVoltage"
        )
    }

    public fun setVoltages(left: Voltage, right: Voltage)

    public var inverted: Boolean

    public fun updateInputs(inputs: Inputs)
}