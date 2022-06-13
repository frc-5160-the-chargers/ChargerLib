package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem

public fun InstantCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand =
    InstantCommand(toRun, *subsystems)