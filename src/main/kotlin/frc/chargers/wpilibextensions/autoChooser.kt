package frc.chargers.wpilibextensions

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command

/**
 * A convenience function to create an auto command chooser in a more concise way.
 */
public inline fun autoChooser(
    name: String = "Auto Command",
    block: SendableChooser<Command>.() -> Unit
): SendableChooser<Command> = SendableChooser<Command>()
    .apply(block)
    .also {
        SmartDashboard.putData(name, it)
    }