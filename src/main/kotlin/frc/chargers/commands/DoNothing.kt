package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.CommandBase

/**
 * A placeholder command that does nothing and does not end unless interrupted.
 */
public class DoNothing : CommandBase() {

    override fun initialize() {
        println("WARNING: The DoNothing command is currently scheduled.")
        println("This should only happen if there is an error, or if we don't have an auto(hopefully not lol).")
    }
    override fun isFinished(): Boolean {
        return false
    }
}