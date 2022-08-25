package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem

/**
 * A utility function for creating [InstantCommand]s in a more kotlin-friendly way.
 */
public fun InstantCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand =
    InstantCommand(toRun, *subsystems)