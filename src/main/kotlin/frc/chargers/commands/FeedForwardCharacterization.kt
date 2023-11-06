package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.utils.characterization.FeedForwardCharacterization
import frc.chargers.utils.characterization.FeedForwardCharacterizationData

public fun characterizeFFAngular(
    name: String,
    forwards: Boolean = true,
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


public fun EncoderHolonomicDrivetrain.characterizeDriveMotors(
    forwards: Boolean = true, vararg requirements: Subsystem
): FeedForwardCharacterization =
        object: FeedForwardCharacterization(
            forwards,
            FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
            FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
            {leftVolts: Double, rightVolts: Double ->
                topLeft.io.setDriveVoltage(leftVolts.ofUnit(volts))
                bottomLeft.io.setDriveVoltage(leftVolts.ofUnit(volts))
                topRight.io.setDriveVoltage(rightVolts.ofUnit(volts))
                bottomRight.io.setDriveVoltage(rightVolts.ofUnit(volts))

                topLeft.setDirection(0.degrees)
                topRight.setDirection(0.degrees)
                bottomLeft.setDirection(0.degrees)
                bottomRight.setDirection(0.degrees)
            },
            { (topLeft.currentVelocity + bottomLeft.currentVelocity).siValue / 2.0 },
            { (topRight.currentVelocity + bottomRight.currentVelocity).siValue / 2.0 },
            this, *requirements
        ){

            override fun initialize(){
                println("ANGULAR DRIVETRAIN characterization is starting")
                println("To stop the characterization, the command must be manually stopped!")
                super.initialize()
            }


            override fun end(interrupted: Boolean) {
                super.end(interrupted)
                println("Voltage UNIT: VOLTS")
                println("Angle UNIT: RADIANS")
                println("Time UNIT: SECONDS")
            }
        }

public fun EncoderHolonomicDrivetrain.characterizeTurnMotors(vararg requirements: Subsystem): Command =
    buildCommand{
        printToConsole{"Swerve TURN motor characterization is starting!"}

        runParallelUntilAllFinish{
            +characterizeFFAngular(
                "TOP LEFT turn motor data",
                true,
                {topLeft.io.setTurnVoltage(it)},
                {topLeft.currentVelocity}
            )

            +characterizeFFAngular(
                "TOP RIGHT turn motor data",
                true,
                {topRight.io.setTurnVoltage(it)},
                {topRight.currentVelocity}
            )

            +characterizeFFAngular(
                "BOTTOM LEFT turn motor data",
                true,
                {bottomLeft.io.setTurnVoltage(it)},
                {bottomLeft.currentVelocity}
            )

            +characterizeFFAngular(
                "BOTTOM RIGHT turn motor data",
                true,
                {bottomRight.io.setTurnVoltage(it)},
                {bottomRight.currentVelocity}
            )
        }
    }.finallyDo{
        println("Voltage UNIT: VOLTS")
        println("Angle UNIT: RADIANS")
        println("Time UNIT: SECONDS")
    }.also{
        it.addRequirements(this,*requirements)
    }



public fun EncoderDifferentialDrivetrain.characterize(
    forwards: Boolean = true, vararg requirements: Subsystem
): FeedForwardCharacterization =
    object: FeedForwardCharacterization(
        forwards,
        FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
        FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
        {leftVolts: Double, rightVolts -> io.setVoltages(leftVolts.volts, rightVolts.volts)},
        {inputs.leftAngularVelocity.siValue},
        {inputs.rightAngularVelocity.siValue},
        this, *requirements
    ){

        override fun initialize(){
            println("ANGULAR DRIVETRAIN characterization is starting.")
            println("To stop the characterization, the command must be manually stopped!")
            super.initialize()
        }


        override fun end(interrupted: Boolean) {
            super.end(interrupted)
            println("Voltage UNIT: VOLTS")
            println("Angle UNIT: RADIANS")
            println("Time UNIT: SECONDS")
        }
    }