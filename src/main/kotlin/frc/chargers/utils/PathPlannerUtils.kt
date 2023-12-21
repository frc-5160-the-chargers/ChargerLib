package frc.chargers.utils

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints

/**
 * Converts ChargerLib PIDConstants to PathPlanner's PIDConstants.
 */
public fun PIDConstants.asPathPlannerConstants(): com.pathplanner.lib.util.PIDConstants =
    com.pathplanner.lib.util.PIDConstants(kP, kI, kD)

/**
 * Represents Data pertinent to PathPlanner autonomous routines.
 *
 * @param translationConstants: The translation PID constants used to converge to the path.
 * @param rotationConstants: The rotation PID constants used to converge to the path.
 * @param constraints: The primary path constraints for PathPlanner. Represented as a [LinearMotionConstraints].
 * @param otherConstraints: The other PathConstraints pertinent.
 * This is useful if you have multiple path constraints for multiple paths within a path group.
 */
public data class PathData(
    val translationConstants: PIDConstants,
    val rotationConstants: PIDConstants,
    val constraints: LinearMotionConstraints,
    val otherConstraints: List<LinearMotionConstraints>
){
    public constructor(
        translationConstants: PIDConstants,
        rotationConstants: PIDConstants,
        constraints: LinearMotionConstraints,
        vararg otherConstraints: LinearMotionConstraints
    ): this(
        translationConstants,
        rotationConstants,
        constraints,
        listOf(*otherConstraints)
    )
}


@DslMarker
public annotation class PathPlannerAutoContextMarker

/**
 * Used to create the HashMap nessecary for Path Planner auto events.
 *
 * Essentially, we can now do this:
 *
 *  {
 *      onEvent("example", command)
 *      onEvent("example"){
 *          runOnce(drivetrain){...}
 *      }
 *  }
 * Instead of:
 *
 * val mutableMap = ("example" to command, onEvent to buildCommand{...})
 *
 */
@PathPlannerAutoContextMarker
public class PathPlannerAutoContext{
    public var eventMap: MutableMap<String, Command> = mutableMapOf()

    public val javaEventMap: HashMap<String, Command>
        get() = HashMap(eventMap)
    public fun onEvent(eventName: String, command: Command){
        eventMap[eventName] = command
    }

    public inline fun onEvent(
        eventName: String,
        context: CommandBuilder.() -> Unit
    ): Unit = onEvent(eventName, buildCommand(block = context))
}