package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import frc.chargers.wpilibextensions.motorcontrol.setVoltage

public interface EncoderDifferentialDrivetrainIO{
    public class Inputs: ChargerLoggableInputs(){
        public var leftAngularPosition: Angle by loggedQuantity(
            Angle(0.0),
            "leftPositionDeg",
            degrees
        )
        public var rightAngularPosition: Angle by loggedQuantity(
            Angle(0.0),
            "leftPositionDeg",
            degrees
        )

        public var leftAngularVelocity: AngularVelocity by loggedQuantity(
            AngularVelocity(0.0),
            "leftVelocityDegPerSecond",
            degrees / seconds
        )
        public var rightAngularVelocity: AngularVelocity by loggedQuantity(
            AngularVelocity(0.0),
            "rightVelocityDegPerSecond",
            degrees / seconds
        )
    }

    public fun setVoltages(left: Voltage, right: Voltage)

    public var inverted: Boolean

    public fun updateInputs(inputs: Inputs)
}

public class EncoderDifferentialDrivetrainIOReal(
    private val leftMotors: NonConfigurableEncoderMotorControllerGroup,
    private val rightMotors: NonConfigurableEncoderMotorControllerGroup
): EncoderDifferentialDrivetrainIO{

    init {
        leftMotors.inverted = false
        rightMotors.inverted = true

    }
    override fun setVoltages(left: Voltage, right: Voltage) {
        // uses custom extension functions; see wpilibextensions
        leftMotors.setVoltage(left)
        rightMotors.setVoltage(right)
    }

    override fun updateInputs(inputs: EncoderDifferentialDrivetrainIO.Inputs){
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

public class EncoderDifferentialDrivetrainIOSim: EncoderDifferentialDrivetrainIO{
    private val sim: DifferentialDrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
        DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide,
        DifferentialDrivetrainSim.KitbotGearing.k10p71, DifferentialDrivetrainSim.KitbotWheelSize.kSixInch, null
    )

    // gearRatio * wheelDiameter for simulator
    private val wheelTravelPerMotorRadian = 10.71 * 6.inches
    override fun setVoltages(left: Voltage, right: Voltage) {
        if (inverted){
            sim.setInputs(
                left.coerceIn(-12.volts,12.volts).inUnit(volts),
                right.coerceIn(-12.volts,12.volts).inUnit(volts)
            )
        }else{
            sim.setInputs(
                -left.coerceIn(-12.volts,12.volts).inUnit(volts),
                -right.coerceIn(-12.volts,12.volts).inUnit(volts)
            )
        }
    }

    override fun updateInputs(inputs: EncoderDifferentialDrivetrainIO.Inputs) {
        sim.update(0.02)
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