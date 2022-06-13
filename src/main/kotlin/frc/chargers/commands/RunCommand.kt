package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem

public fun RunCommand(vararg subsystems: Subsystem, toRun: () -> Unit): RunCommand =
    RunCommand(toRun, *subsystems)