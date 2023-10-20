package frc.chargers.framework

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.hardware.subsystems.TunableSubsystem

/**
 * Represents a basic Robot Container, for use in Command-Based programming.
 */
public abstract class ChargerRobotContainer {

    public fun setTuningMode(status: Boolean){
        TunableSubsystem.tuningMode = status
    }


    public abstract val autonomousCommand: Command

    // instantCommand is used over DoNothing in case the test command isn't canceled for some reason
    public val testCommand: Command get() = InstantCommand{}

    /*
    Here are functions that replicate functionality of the Robot class.
     */
    public open fun disabledInit() {}
    public open fun disabledPeriodic() {}
    public open fun autonomousInit() {}
    public open fun autonomousPeriodic() {}
    public open fun teleopInit() {}
    public open fun teleopPeriodic() {}
    public open fun testInit() {}
    public open fun testPeriodic() {}
    public open fun simulationInit() {}
    public open fun simulationPeriodic() {}

}