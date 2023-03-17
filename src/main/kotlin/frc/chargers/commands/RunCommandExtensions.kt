package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

/**
 * A utility function for creating [RunCommand]s in a more kotlin-friendly way.
 */
public fun RunCommand(vararg subsystems: Subsystem, toRun: () -> Unit): RunCommand =
    RunCommand(toRun, *subsystems)

/* 
 *   An alternative way to define RunCommands that is more in line with the buildCommand commands.
 */
public fun RunForeverCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand = 
    RunCommand(*subsystems,toRun)

/**
 * A utility function for setting the default [Command] of a [Subsystem] to a [RunCommand]
 * in a shorter and easier way.
 *
 * @see Subsystem.setDefaultCommand
 */
public fun Subsystem.setDefaultRunCommand(vararg requirements: Subsystem = arrayOf(this), toRun: () -> Unit) {
    defaultCommand = RunCommand(toRun, *requirements)
}
