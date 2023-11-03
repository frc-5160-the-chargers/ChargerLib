package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.utils.characterization.FeedForwardCharacterization
import frc.chargers.utils.characterization.FeedForwardCharacterizationData

public fun characterizeFFAngular(
    name: String,
    forwards: Boolean,
    setVoltage: (Voltage) -> Unit,
    getVelocity: () -> AngularVelocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards, FeedForwardCharacterizationData(name), { inputVolts -> setVoltage(inputVolts.ofUnit(volts))},
    {getVelocity().siValue},
    *requirements
){

    override fun initialize(){
        println("ANGULAR characterization is starting")
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: radians")
        println("Time UNIT: seconds")
    }
}

public fun characterizeFFLinear(
    name: String,
    forwards: Boolean,
    setVoltage: (Voltage) -> Unit,
    getVelocity: () -> Velocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards, FeedForwardCharacterizationData(name), { inputVolts -> setVoltage(inputVolts.ofUnit(volts))},
    {getVelocity().siValue},
    *requirements
){

    override fun initialize(){
        println("LINEAR characterization is starting")
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: METERS")
        println("Time UNIT: SECONDS")
    }
}


public fun characterizeDriveFFAngular(
    forwards: Boolean,
    setVoltageLeft: (Voltage) -> Unit,
    setVoltageRight: (Voltage) -> Unit,
    getVelocityLeft: () -> AngularVelocity,
    getVelocityRight: () -> AngularVelocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards,
    FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
    FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
    {leftVolts: Double, rightVolts -> setVoltageLeft(leftVolts.ofUnit(volts)); setVoltageRight(rightVolts.ofUnit(volts))},
    {getVelocityLeft().siValue},
    {getVelocityRight().siValue},
     *requirements
){

    override fun initialize(){
        println("ANGULAR DRIVETRAIN characterization is starting")
        super.initialize()
    }


    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: RADIANS")
        println("Time UNIT: SECONDS")
    }
}

public fun characterizeDriveFFLinear(
    forwards: Boolean,
    setVoltageLeft: (Voltage) -> Unit,
    setVoltageRight: (Voltage) -> Unit,
    getVelocityLeft: () -> Velocity,
    getVelocityRight: () -> Velocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards,
    FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
    FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
    {leftVolts: Double, rightVolts -> setVoltageLeft(leftVolts.ofUnit(volts)); setVoltageRight(rightVolts.ofUnit(volts))},
    {getVelocityLeft().siValue},
    {getVelocityRight().siValue},
    *requirements
){
    override fun initialize(){
        println("LINEAR DRIVETRAIN characterization is starting")
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: RADIANS")
        println("Time UNIT: SECONDS")
    }
}