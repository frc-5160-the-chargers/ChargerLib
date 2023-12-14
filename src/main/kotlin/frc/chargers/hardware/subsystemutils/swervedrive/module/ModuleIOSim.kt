package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.constants.drivetrain.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.constants.drivetrain.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.math.units.Inertia
import frc.chargers.utils.math.units.times

public class ModuleIOSim(
    logInputs: LoggableInputsProvider,
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA
): ModuleIO {
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
    private var turnAppliedVoltage = Voltage(0.0)
    private var driveAppliedVoltage = Voltage(0.0)

    init{
        ChargerRobot.runPeriodically(addToFront = true){
            turnMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            driveMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            currentDirection += turnMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds) * ChargerRobot.LOOP_PERIOD
            currentWheelPosition += driveMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds) * ChargerRobot.LOOP_PERIOD
        }
    }



    override val logTab: String = logInputs.namespace

    override val direction: Angle by logInputs.quantity{
        currentDirection.inputModulus(0.degrees..360.degrees)
    }

    override val turnSpeed: AngularVelocity by logInputs.quantity{
        turnMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val speed: AngularVelocity by logInputs.quantity{
        driveMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val wheelTravel: Angle by logInputs.quantity{currentWheelPosition}

    override var turnVoltage: Voltage by logInputs.quantity(
        getValue = {turnAppliedVoltage},
        setValue = {
            turnAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            turnMotorSim.setInputVoltage(it.inUnit(volts))
        }
    )

    override var driveVoltage: Voltage by logInputs.quantity(
        getValue = {driveAppliedVoltage},
        setValue = {
            driveAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            driveMotorSim.setInputVoltage(driveAppliedVoltage.inUnit(volts))
        }
    )

}