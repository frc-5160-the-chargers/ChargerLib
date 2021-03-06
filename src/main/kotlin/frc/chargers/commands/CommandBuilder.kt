package frc.chargers.commands

import com.batterystaple.kmeasure.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.*
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/**
 * The entry point for the CommandBuilder DSL (Domain Specific Language).
 *
 * See [here](https://kotlinlang.org/docs/type-safe-builders.html#how-it-works)
 * for an explanation of DSLs and how they are built.
 */
public inline fun buildCommand(block: CommandBuilder.() -> Unit): Command =
    SequentialCommandGroup(*CommandBuilder().apply(block).commands.toTypedArray())


@DslMarker
public annotation class CommandBuilderMarker

@CommandBuilderMarker
public class CommandBuilder {
    @PublishedApi
    internal var commands: LinkedHashSet<Command> = linkedSetOf() // LinkedHashSet keeps commands in order, but also ensures they're not added multiple times

    /**
     * Adds a single command to be run until its completion.
     */
    public fun <C : Command> runUntilFinish(command: C): C {
        commands.add(command)
        return command
    }

    /**
     * Adds a command that will run once and then complete.
     *
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run
     */
    public fun runOnce(vararg requirements: Subsystem, execute: CodeBlockContext.() -> Unit): InstantCommand =
        InstantCommand(*requirements) { CodeBlockContext.execute() }.also(commands::add)

    /**
     * Adds an InstantCommand that will run once and then complete.
     *
     * @see runUntilFinish
     */
    public fun runOnce(command: InstantCommand): InstantCommand =
        runUntilFinish(command)

    /**
     * Adds a command that will run *until* [condition] is met.
     *
     * @param condition the condition to be met
     * @param command the command to run until [condition] is met
     */
    public fun runUntil(condition: CodeBlockContext.() -> Boolean, command: Command): ParallelRaceGroup =
        command.until { CodeBlockContext.condition() }
            .also(commands::add)

    /**
     * Adds a command that will run *until* [condition] is met.
     *
     * @param condition the condition to be met
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run until [condition] is met
     */
    public inline fun runUntil(noinline condition: CodeBlockContext.() -> Boolean, vararg requirements: Subsystem, crossinline execute: CodeBlockContext.() -> Unit): ParallelRaceGroup =
        runUntil(condition, RunCommand(*requirements) { CodeBlockContext.execute() })

    /**
     * Adds a command that will run *while* [condition] is true.
     *
     * @param condition the condition to be met
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run
     */
    public inline fun runWhile(crossinline condition: CodeBlockContext.() -> Boolean, vararg requirements: Subsystem, noinline execute: CodeBlockContext.() -> Unit): ParallelRaceGroup =
        runUntil({ !condition() }, *requirements, execute=execute)

    /**
     * Adds several commands that will run at the same time, all stopping as soon as one finishes.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelRaceGroup
     */
    public fun runParallelUntilOneFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit): ParallelRaceGroup {
        val commandsSet = commands.toMutableSet() + CommandBuilder().apply(block).commands
        this.commands.removeAll(commandsSet)
        return ParallelRaceGroup(*commandsSet.toTypedArray()).also(this.commands::add)
    }

    /**
     * Adds several commands that will run at the same time, only finishing once all are complete.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelCommandGroup
     */
    public fun runParallelUntilAllFinish(vararg commands: Command, block: CommandBuilder.() -> Unit): ParallelCommandGroup {
        val commandsSet = commands.toMutableSet() + CommandBuilder().apply(block).commands
        this.commands.removeAll(commandsSet)
        return ParallelCommandGroup(*commandsSet.toTypedArray()).also(this.commands::add)
    }

    /**
     * Adds several commands that will run one after another.
     *
     * @param block a builder to create the commands to run sequentially
     * @see SequentialCommandGroup
     */
    public fun runSequentially(block: CommandBuilder.() -> Unit): SequentialCommandGroup {
        return SequentialCommandGroup(*CommandBuilder().apply(block).commands.toTypedArray()).also(this.commands::add)
    }

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param command the command to run
     * @param timeInterval the maximum allowed runtime of the command
     */
    public fun runFor(timeInterval: Time, command: Command): ParallelRaceGroup {
        return command
            .withTimeout(timeInterval.inUnit(Seconds))
            .also(commands::add)
    }

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param timeInterval the maximum allowed runtime of the command
     * @param requirements the Subsystems this command requires
     * @param execute the code to be run
     */
    public inline fun runFor(timeInterval: Time, vararg requirements: Subsystem, crossinline execute: CodeBlockContext.() -> Unit): ParallelRaceGroup =
        runFor(timeInterval, RunCommand(*requirements) { CodeBlockContext.execute() })

    /**
     * Adds a command to be run continuously.
     *
     * @param the Subsystems this command requires
     * @param execute the code to be run
     */
    public fun runForever(vararg requirements: Subsystem, block: CodeBlockContext.() -> Unit): RunCommand =
            RunCommand(*requirements) { CodeBlockContext.block() }
                .also(commands::add)

    /**
     * Adds a command that does nothing for a specified [timeInterval], then completes.
     *
     * Useful if a delay is needed between two commands in a [SequentialCommandGroup].
     * Note that running this in parallel with other commands is unlikely to be useful.
     */
    public fun waitFor(timeInterval: Time): ParallelRaceGroup = runFor(timeInterval) {}

    /**
     * Adds a command that does nothing until a [condition] is met, then completes.
     *
     * Useful if some condition must be met before proceeding to the next command in a [SequentialCommandGroup].
     * Note that running this in parallel with other commands is unlikely to be useful.
     */
    public fun waitUntil(condition: CodeBlockContext.() -> Boolean): ParallelRaceGroup = runUntil(condition) {}

    /**
     * Adds a command that prints a message.
     *
     * @param message a function that generates the message to print
     */
    public fun printToConsole(message: () -> Any?): Command =
        InstantCommand { println(message()) }.also(commands::add)

    /**
     * Adds a command that gets a property when it executes; this is useful for
     * getting initial positions, etc. when commands first run.
     *
     * Returns a property delegate; see [here](https://kotlinlang.org/docs/delegated-properties.html#standard-delegates)
     * for an explanation of property delegates.
     */
    public fun <T : Any> getOnceDuringRun(get: CodeBlockContext.() -> T) : ReadOnlyProperty<Any?, T> =
        DuringRunGetter(get)

    private inner class DuringRunGetter<T : Any>(private val get: CodeBlockContext.() -> T) : ReadOnlyProperty<Any?, T> {
        init {
            commands.add(
                object : CommandBase() { // Add a new command that initializes this value in its initialize() function.
                    override fun initialize() {
                        if (!::value.isInitialized) {
                            initializeValue()
                        }
                    }

                    override fun isFinished(): Boolean = ::value.isInitialized
                }
            )
        }

        private fun initializeValue() {
            value = CodeBlockContext.get()
        }

        private lateinit var value: T

        override fun getValue(thisRef: Any?, property: KProperty<*>): T =
            try {
                value
            } catch (e: UninitializedPropertyAccessException) { // If value is tried to be used before the initializer command runs (for example, by another command running in parallel), then initialize it immediately.
                initializeValue()
                value
            }
    }
}

/**
 * Adds various utility properties and functions to the local context.
 */
@CommandBuilderMarker
public interface TimeContext {
    public val time: Time get() = Timer.getFPGATimestamp().ofUnit(Seconds)
    public val approximateTimeSinceMatchStart: Time get() = Timer.getMatchTime().ofUnit(Seconds)
    public fun delay(time: Time) { Timer.delay(time.inUnit(Seconds)) }

    public companion object : TimeContext
}

/**
 * The context provided to any code block in a CommandBuilder,
 * allowing code blocks to access various utility properties and functions.
 */
@CommandBuilderMarker
public object CodeBlockContext : TimeContext