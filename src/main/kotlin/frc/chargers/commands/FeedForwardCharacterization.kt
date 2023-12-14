package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargerlibexternal.frc6328.characterization.FeedForwardCharacterization
import frc.chargerlibexternal.frc6328.characterization.FeedForwardCharacterizationData

public fun characterizeFFAngular(
    name: String,
    forwards: Boolean = true,
    setVoltage: (Voltage) -> Unit,
    getVelocity: () -> AngularVelocity,
    vararg requirements: Subsystem
): FeedForwardCharacterization = object: FeedForwardCharacterization(
    forwards,
    FeedForwardCharacterizationData(name), { inputVolts -> setVoltage(inputVolts.ofUnit(volts))},
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
    forwards,
    FeedForwardCharacterizationData(name), { inputVolts -> setVoltage(inputVolts.ofUnit(volts))},
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
                topLeft.io.driveVoltage = leftVolts.ofUnit(volts)
                bottomLeft.io.driveVoltage = leftVolts.ofUnit(volts)
                topRight.io.driveVoltage = rightVolts.ofUnit(volts)
                bottomRight.io.driveVoltage = rightVolts.ofUnit(volts)

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

        runParallelUntilAllFinish{
            +characterizeFFAngular(
                "TOP LEFT turn motor data",
                true,
                {topLeft.io.turnVoltage = it; println(it)},
                {topLeft.currentTurningVelocity}
            )

            +characterizeFFAngular(
                "TOP RIGHT turn motor data",
                true,
                {topRight.io.turnVoltage = it},
                {topRight.currentTurningVelocity}
            )

            +characterizeFFAngular(
                "BOTTOM LEFT turn motor data",
                true,
                {bottomLeft.io.turnVoltage = it},
                {bottomLeft.currentTurningVelocity}
            )

            +characterizeFFAngular(
                "BOTTOM RIGHT turn motor data",
                true,
                {bottomRight.io.turnVoltage = it},
                {bottomRight.currentTurningVelocity}
            )
        }
    }.withExtraRequirements(this@characterizeTurnMotors, *requirements)


public fun EncoderHolonomicDrivetrain.driveTurnMotors(): Command = buildCommand{
    runParallelUntilAllFinish{
        loopForever{
            topLeft.io.turnVoltage = 2.0.volts
        }

        loopForever{
            topRight.io.turnVoltage = 2.0.volts
        }

        loopForever{
            bottomLeft.io.turnVoltage = 2.0.volts
        }

        loopForever{
            bottomRight.io.turnVoltage = 2.0.volts
        }

    }
}


public fun EncoderDifferentialDrivetrain.characterize(
    forwards: Boolean = true, vararg requirements: Subsystem
): FeedForwardCharacterization =
    object: FeedForwardCharacterization(
        forwards,
        FeedForwardCharacterizationData("SwerveDriveFFData_Left"),
        FeedForwardCharacterizationData("SwerveDriveFFData_Right"),
        {leftVolts: Double, rightVolts -> io.setVoltages(leftVolts.volts, rightVolts.volts)},
        {io.leftVelocity.siValue},
        {io.rightVelocity.siValue},
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


/*

context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderDifferentialDrivetrain.characterize(
    forwards: Boolean = true, vararg requirements: Subsystem
): Command = characterize(forwards,*requirements).also(commands::add)

context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.characterizeTurnMotors(vararg requirements: Subsystem): Command =
    characterizeTurnMotors(*requirements).also(commands::add)

context(CommandBuilder)
@LowPriorityInOverloadResolution
public fun EncoderHolonomicDrivetrain.characterizeDriveMotors(
    forwards: Boolean = true, vararg requirements: Subsystem
): Command = characterizeDriveMotors(forwards,*requirements).also(commands::add)


 */
