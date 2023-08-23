package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Velocity
import com.pathplanner.lib.PathConstraints
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.CommandBuilder
import frc.chargers.commands.buildCommand
import frc.chargers.controls.pid.PIDConstants

/**
 * Converts ChargerLib PIDConstants to PathPlanner's PIDConstants.
 */
public fun PIDConstants.asPathPlannerConstants(): com.pathplanner.lib.auto.PIDConstants =
    com.pathplanner.lib.auto.PIDConstants(kP, kI, kD)


public fun PathConstraints(maxVelocity: Velocity, maxAcceleration: Acceleration): PathConstraints = PathConstraints(
    maxVelocity.siValue,
    maxAcceleration.siValue
)

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
    ): Unit = onEvent(eventName, buildCommand(context))
}