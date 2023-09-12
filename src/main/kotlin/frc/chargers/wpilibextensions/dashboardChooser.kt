package frc.chargers.wpilibextensions

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.DoNothing
import frc.chargers.utils.MappableContext

public inline fun <I> dashboardChooser(
    name: String,
    defaultMap: Pair<String,I>,
    block: MappableContext<String, I>.(SendableChooser<I>) -> Unit
): SendableChooser<I> {
    val chooser = SendableChooser<I>()
    val map = MappableContext<String, I>().apply{
        block(chooser)
    }.map

    chooser.setDefaultOption(defaultMap.first, defaultMap.second)

    map.keys.forEach{key ->
        chooser.addOption(key,map[key])
    }
    SmartDashboard.putData(name,chooser)
    return chooser
}