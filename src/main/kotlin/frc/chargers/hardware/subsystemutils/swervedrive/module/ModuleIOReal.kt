package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.motorcontrol.setVoltage

public class ModuleIOReal(
    logInputs: LoggableInputsProvider,
    private val turnMotor: EncoderMotorController,
    private val turnEncoder: PositionEncoder,
    private val driveMotor: EncoderMotorController,
    private val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    private val turnGearRatio: Double = DEFAULT_GEAR_RATIO
): ModuleIO {
    private val startingWheelTravel = driveMotor.encoder.angularPosition
    private var turnAppliedVoltage = Voltage(0.0)
    private var driveAppliedVoltage = Voltage(0.0)



    override val logTab: String = logInputs.namespace

    override val direction: Angle by logInputs.quantity{
        turnEncoder.angularPosition.inputModulus(0.degrees..360.degrees)
    }
    override val turnSpeed: AngularVelocity by logInputs.quantity{
        turnMotor.encoder.angularVelocity * turnGearRatio
    }

    override val speed: AngularVelocity by logInputs.quantity{
        driveMotor.encoder.angularVelocity * driveGearRatio
    }
    override val wheelTravel: Angle by logInputs.quantity{
        (driveMotor.encoder.angularPosition - startingWheelTravel) * driveGearRatio
    }

    override var turnVoltage: Voltage by logInputs.quantity(
        getValue = {turnAppliedVoltage},
        setValue = {
            turnAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            turnMotor.setVoltage(turnAppliedVoltage)
        }
    )
    override var driveVoltage: Voltage by logInputs.quantity(
        getValue = {driveAppliedVoltage},
        setValue = {
            driveAppliedVoltage = it.coerceIn(-12.volts..12.volts)
            driveMotor.setVoltage(driveAppliedVoltage)
        }
    )

}