package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.constants.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.constants.drivetrain.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.constants.drivetrain.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.math.units.Inertia
import frc.chargers.utils.math.units.times
import frc.chargers.wpilibextensions.motorcontrol.setVoltage


public class ModuleIOReal(
    logTab: LoggableInputsProvider,
    private val turnMotor: EncoderMotorController,
    private val turnEncoder: PositionEncoder,
    private val driveMotor: EncoderMotorController,
    private val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    private val turnGearRatio: Double = DEFAULT_GEAR_RATIO
): ModuleIO {
    override val logTab: String = logTab.logGroup

    private val startingWheelTravel = driveMotor.encoder.angularPosition

    override val direction: Angle by logTab.quantity{ turnEncoder.angularPosition.inputModulus(0.degrees..360.degrees) }
    override val turnSpeed: AngularVelocity by logTab.quantity{ turnMotor.encoder.angularVelocity / turnGearRatio }

    override val speed: AngularVelocity by logTab.quantity{ driveMotor.encoder.angularVelocity / driveGearRatio }
    override val wheelTravel: Angle by logTab.quantity{ (driveMotor.encoder.angularPosition - startingWheelTravel) / driveGearRatio }

    private var turnAppliedVoltage = Voltage(0.0)
    private var driveAppliedVoltage = Voltage(0.0)

    override var turnVoltage: Voltage by logTab.quantity(
        getValue = {turnAppliedVoltage},
        setValue = {
            turnAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            turnMotor.setVoltage(turnAppliedVoltage)
        }
    )
    override var driveVoltage: Voltage by logTab.quantity(
        getValue = {driveAppliedVoltage},
        setValue = {
            driveAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            driveMotor.setVoltage(driveAppliedVoltage)
        }
    )

}


public class ModuleIOSim(
    logTab: LoggableInputsProvider,
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA
): ModuleIO {

    override val logTab: String = logTab.logGroup

    private val turnMotorSim = FlywheelSim(
        turnGearbox,
        1 / turnGearRatio,
        turnInertiaMoment.inUnit(kilo.grams * meters * meters)
    )
    private val driveMotorSim = FlywheelSim(
        driveGearbox,
        1 / driveGearRatio,
        driveInertiaMoment.inUnit(kilo.grams * meters * meters)
    )
    private var currentDirection = Angle(0.0)
    private var currentWheelPosition = Angle(0.0)

    init{
        ChargerRobot.runPeriodically(addToFront = true){
            turnMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            driveMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            currentDirection += turnMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds) * ChargerRobot.LOOP_PERIOD
            currentWheelPosition += driveMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds) * ChargerRobot.LOOP_PERIOD
        }
    }




    override val direction: Angle by logTab.quantity{
        currentDirection.inputModulus(0.degrees..360.degrees)
    }
    override val turnSpeed: AngularVelocity by logTab.quantity{
        turnMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)
    }

    override val speed: AngularVelocity by logTab.quantity{
        driveMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)
    }
    override val wheelTravel: Angle by logTab.quantity{currentWheelPosition}

    private var turnAppliedVoltage = Voltage(0.0)
    private var driveAppliedVoltage = Voltage(0.0)

    override var turnVoltage: Voltage by logTab.quantity(
        getValue = {turnAppliedVoltage},
        setValue = {
            turnAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            turnMotorSim.setInputVoltage(it.inUnit(volts))
        }
    )

    override var driveVoltage: Voltage by logTab.quantity(
        getValue = {driveAppliedVoltage},
        setValue = {
            driveAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            driveMotorSim.setInputVoltage(driveAppliedVoltage.inUnit(volts))
        }
    )

}

public interface ModuleIO{
    public val logTab: String


    public val direction: Angle
    public val turnSpeed: AngularVelocity

    public val speed: AngularVelocity
    public val wheelTravel: Angle

    public var turnVoltage: Voltage
    public var driveVoltage: Voltage
}


