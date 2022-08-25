package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.CommandBase

/**
 * A placeholder command that does nothing and does not end unless interrupted.
 */
public class DoNothing : CommandBase() {
    override fun isFinished(): Boolean {
        return false
    }
}