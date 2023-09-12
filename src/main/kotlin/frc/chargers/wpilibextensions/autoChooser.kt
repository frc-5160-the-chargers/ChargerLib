package frc.chargers.wpilibextensions

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.DoNothing
import frc.chargers.utils.MappableContext

/**
 * A convenience function to create an auto command chooser in a more concise way.
 * The context of the block is [MappableContext],
 *
 * however, you can still access the [SendableChooser] using the term 'it'.
 *
 * Usage:
 * ```
 * val chooser = autoChooser(defaultKey = "Do Nothing"){
 *      "Do Nothing" to DoNothing()
 *      "Drive Backwards" to buildCommand{
 *          drivetrain.driveStraight(...)
 *      }
 *      "Follow Path" to buildCommand{
 *          drivetrain.followPath(...)
 *      }
 * }
 * ```
 */
public inline fun autoChooser(
    defaultKey: String,
    name: String = "Auto Command",
    block: MappableContext<String,Command>.(SendableChooser<Command>) -> Unit
): SendableChooser<Command>{
    val chooser = SendableChooser<Command>()
    val map = MappableContext<String,Command>().apply{
        block(chooser)
    }.map

    var defaultCommandAssigned = false
    map.keys.forEach{key ->
        if (key == defaultKey){
            chooser.setDefaultOption(key,map[key])
            defaultCommandAssigned = true
        }else{
            chooser.addOption(key,map[key])
        }
    }
    if (!defaultCommandAssigned){
        chooser.setDefaultOption(defaultKey + "(Do Nothing); default routine was not set by chooser!", DoNothing())
        println("WARNING: Auto chooser DID NOT set a default autonomous routine. It has been defaulted to DoNothing().")
    }
    SmartDashboard.putData(name,chooser)
    return chooser
}