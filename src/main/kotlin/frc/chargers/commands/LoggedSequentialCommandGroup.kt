package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.littletonrobotics.junction.Logger


/**
 * Utility functions used for logging commands within the buildCommand DSL.
 */

@PublishedApi
internal fun loggedSequentialCommandGroup(name: String, vararg commands: Command): SequentialCommandGroup{
    val loggedCommands: Array<Command> = commands.map{it.withLogInCommandGroup(name)}.toTypedArray()
    return SequentialCommandGroup(
        *loggedCommands
    ).also{
        it.name = name
    }
}

internal fun Command.withLogInCommandGroup(commandGroupName: String): Command{


    fun logCommand(active: Boolean) = InstantCommand{
        Logger.getInstance().recordOutput(
            "/ActiveCommands/$commandGroupName-subcommands/$name",active
        )
    }

    // uses custom infix "then" operator(more concise way to do andThen)
    return logCommand(true) then this.withStartEndLog() then logCommand(false)
}
