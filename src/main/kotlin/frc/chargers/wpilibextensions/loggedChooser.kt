package frc.chargers.wpilibextensions

import frc.chargers.utils.MappableContext
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * A wrapper function around [LoggedDashboardChooser], using [MappableContext].
 */
public inline fun <I> loggedChooser(
    name: String,
    defaultKey: String,
    block: MappableContext<String, I>.(LoggedDashboardChooser<I>) -> Unit
): LoggedDashboardChooser<I> {
    val chooser = LoggedDashboardChooser<I>(name)
    val map = MappableContext<String, I>().apply{
        block(chooser)
    }.map
    map.forEach{ (key, value) ->
        if (key == defaultKey){
            chooser.addDefaultOption(key,value)
        }else{
            chooser.addOption(key,value)
        }
    }
    return chooser
}