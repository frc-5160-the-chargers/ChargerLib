package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

/**
 * A utility function for creating [RunCommand]s in a more kotlin-friendly way.
 */
public fun RunCommand(vararg subsystems: Subsystem, toRun: () -> Unit): RunCommand =
    RunCommand(toRun, *subsystems)

/**
 * A utility function for setting the default [Command] of a [Subsystem] to a [RunCommand]
 * in a shorter and easier way.
 *
 * @see Subsystem.setDefaultCommand
 */
public fun <S: Subsystem> S.setDefaultRunCommand(vararg requirements: Subsystem, toRun: S.() -> Unit){
    defaultCommand = RunCommand({toRun()}, this, *requirements)
}
