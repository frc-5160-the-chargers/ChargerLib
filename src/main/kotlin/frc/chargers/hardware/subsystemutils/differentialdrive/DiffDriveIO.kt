package frc.chargers.hardware.subsystemutils.differentialdrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.NonConfigurableEncoderMotorControllerGroup
import frc.chargers.wpilibextensions.motorcontrol.setVoltage


public class DiffDriveIOReal(
    logTab: LoggableInputsProvider,
    private val leftMotors: NonConfigurableEncoderMotorControllerGroup,
    private val rightMotors: NonConfigurableEncoderMotorControllerGroup
): DiffDriveIO {

    init {
        leftMotors.inverted = false
        rightMotors.inverted = true
    }

    private var leftAppliedVoltage = Voltage(0.0)
    private var rightAppliedVoltage = Voltage(0.0)

    override val leftWheelTravel: Angle by logTab.quantity{leftMotors.encoder.angularPosition}
    override val rightWheelTravel: Angle by logTab.quantity{rightMotors.encoder.angularPosition}

    override val leftVelocity: AngularVelocity by logTab.quantity{leftMotors.encoder.angularVelocity}
    override val rightVelocity: AngularVelocity by logTab.quantity{rightMotors.encoder.angularVelocity}

    override val leftVoltage: Voltage by logTab.quantity{leftAppliedVoltage}
    override val rightVoltage: Voltage by logTab.quantity {rightAppliedVoltage}

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


public class DiffDriveIOSim(
    logTab: LoggableInputsProvider,
    motors: DifferentialDrivetrainSim.KitbotMotor
): DiffDriveIO{
    init{
        ChargerRobot.runPeriodically(addToFront = true){
            sim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        }
    }

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

    override val leftWheelTravel: Angle by logTab.quantity{ sim.leftPositionMeters.ofUnit(meters) / wheelTravelPerMotorRadian }
    override val rightWheelTravel: Angle by logTab.quantity{ sim.rightPositionMeters.ofUnit(meters) / wheelTravelPerMotorRadian }

    override val leftVelocity: AngularVelocity by logTab.quantity{ sim.leftVelocityMetersPerSecond.ofUnit(meters/seconds) / wheelTravelPerMotorRadian }
    override val rightVelocity: AngularVelocity by logTab.quantity{ sim.rightVelocityMetersPerSecond.ofUnit(meters / seconds) / wheelTravelPerMotorRadian }

    override val leftVoltage: Voltage by logTab.quantity{leftAppliedVoltage}
    override val rightVoltage: Voltage by logTab.quantity{rightAppliedVoltage}


    override var inverted: Boolean = false

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

}

public interface DiffDriveIO{
    public val leftWheelTravel: Angle
    public val rightWheelTravel: Angle

    public val leftVelocity: AngularVelocity
    public val rightVelocity: AngularVelocity

    public val leftVoltage: Voltage
    public val rightVoltage: Voltage

    public fun setVoltages(left: Voltage, right: Voltage)

    public var inverted: Boolean
}