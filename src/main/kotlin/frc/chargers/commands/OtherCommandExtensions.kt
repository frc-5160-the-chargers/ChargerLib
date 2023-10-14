package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.*

/**
 * A way to link up 2 commands in a more concise way.
 */
public infix fun Command.then(other: Command): Command =
    andThen(other)

@JvmName("withTimeoutKmeasure")
public fun Command.withTimeout(time: Time): Command = withTimeout(time.inUnit(seconds))

public fun Command.repeatedlyFor(time: Time): Command =
    ParallelRaceGroup(
        repeatedly(),
        WaitCommand(time.inUnit(seconds))
    )

public fun Command.repeatedlyWhile(condition: () -> Boolean): Command = ParallelRaceGroup(
    repeatedly(),
    object: CommandBase(){
        override fun isFinished(): Boolean = condition()
    }
)

public fun Command.repeatFor(numTimes: Int): Command = buildCommand{
    for (i in 1..numTimes){
        +this@repeatFor
    }
}

public fun Command.beforeStarting(vararg subsystems: Subsystem, toRun: () -> Unit): Command =
    beforeStarting(toRun,*subsystems)



