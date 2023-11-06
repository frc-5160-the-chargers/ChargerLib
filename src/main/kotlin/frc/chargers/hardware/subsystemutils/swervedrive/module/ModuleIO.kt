package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.constants.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.constants.drivetrain.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.constants.drivetrain.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.utils.math.units.Inertia
import frc.chargers.wpilibextensions.motorcontrol.voltage
import frc.chargers.utils.math.units.times


public class ModuleIOReal(
    private val turnMotor: EncoderMotorController,
    private val turnEncoder: PositionEncoder,
    private val driveMotor: EncoderMotorController,
    private val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    private val turnGearRatio: Double = DEFAULT_GEAR_RATIO
): ModuleIO {
    override fun setDriveVoltage(driveV: Voltage) {
        // custom extension property
        driveMotor.voltage = driveV
    }

    override fun setTurnVoltage(turnV: Voltage) {
        // custom extension property
        turnMotor.voltage = turnV
    }

    override fun updateInputs(inputs: ModuleIO.Inputs) {
        inputs.apply{
            direction = turnEncoder.angularPosition
            turnSpeed = turnMotor.encoder.angularVelocity * turnGearRatio
            turnVoltage = turnMotor.voltage

            speed = driveMotor.encoder.angularVelocity * driveGearRatio
            driveVoltage = driveMotor.voltage
            distance = driveMotor.encoder.angularPosition * driveGearRatio
        }
    }

}


public class ModuleIOSim(
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: Inertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: Inertia = DEFAULT_SWERVE_DRIVE_INERTIA
): ModuleIO {

    public var driveAppliedVoltage: Voltage = 0.0.volts
        private set
    public var turnAppliedVoltage: Voltage = 0.0.volts
        private set
    private var moduleDirection = 0.0.degrees


    private fun setDirection(value: Angle){
        moduleDirection = value
        while (moduleDirection < 0.degrees){
            moduleDirection += 360.degrees
        }

        while(moduleDirection > 360.degrees){
            moduleDirection -= 360.degrees
        }
    }

    private val turnMotorSim = FlywheelSim(
        turnGearbox,
        1/turnGearRatio,
        turnInertiaMoment.inUnit(kilo.grams * meters * meters)
    )

    private val driveMotorSim = FlywheelSim(
        driveGearbox,
        1/driveGearRatio,
        driveInertiaMoment.inUnit(kilo.grams * meters * meters)
    )

    override fun updateInputs(inputs: ModuleIO.Inputs) {
        turnMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        driveMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))

        val turnVel = turnMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)

        setDirection(moduleDirection + turnVel * ChargerRobot.LOOP_PERIOD)


        inputs.apply{
            direction = moduleDirection
            turnSpeed = turnVel
            turnVoltage = turnAppliedVoltage


            speed = driveMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)
            distance += speed * ChargerRobot.LOOP_PERIOD
            driveVoltage = driveAppliedVoltage
        }

    }


    override fun setDriveVoltage(driveV: Voltage) {
        driveAppliedVoltage = driveV.coerceIn(-12.volts..12.volts)
        driveMotorSim.setInputVoltage(
            driveAppliedVoltage.inUnit(volts)
        )
    }

    override fun setTurnVoltage(turnV: Voltage) {
        turnAppliedVoltage = turnV.coerceIn(-12.volts..12.volts)
        turnMotorSim.setInputVoltage(
            turnAppliedVoltage.inUnit(volts)
        )
    }



}


public interface ModuleIO{
    public class Inputs: ChargerLoggableInputs(){
        public var direction: Angle by loggedQuantity(logUnit = degrees, logName = "directionDegrees")
        public var turnSpeed: AngularVelocity by loggedQuantity(logUnit = degrees/seconds, logName = "turnSpeedDegPerSec")
        public var turnVoltage: Voltage by loggedQuantity(logUnit = volts, logName = "turnVoltageVolts")


        public var speed: AngularVelocity by loggedQuantity(logUnit = degrees/seconds, logName = "speedDegreesPerSec")
        public var driveVoltage: Voltage by loggedQuantity(logUnit = volts, logName = "driveVoltageVolts")
        public var distance: Angle by loggedQuantity(logUnit = degrees, logName = "distanceDegrees")
    }

    public fun setDriveVoltage(driveV: Voltage)
    public fun setTurnVoltage(turnV: Voltage)
    public fun updateInputs(inputs: Inputs)


}