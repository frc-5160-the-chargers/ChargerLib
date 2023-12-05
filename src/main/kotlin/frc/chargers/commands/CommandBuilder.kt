package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior
import frc.chargers.utils.MappableContext
import frc.chargers.utils.a
import org.littletonrobotics.junction.Logger
import kotlin.internal.LowPriorityInOverloadResolution
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/**
 * The entry point for the CommandBuilder DSL (Domain Specific Language).
 *
 * See [here](https://kotlinlang.org/docs/type-safe-builders.html#how-it-works)
 * for an explanation of DSLs and how they are built.
 *
 * @param name The name of the buildCommand(defaults to "Generic BuildCommand").
 * @param logIndividualCommands If true, will log the individual commands that are part of the DSL. Defaults to false.
 * @param block The entry point to the DSL. Has the context of [CommandBuilder].
 */
public inline fun buildCommand(
    name: String = "Generic BuildCommand",
    logIndividualCommands: Boolean = false,
    block: CommandBuilder.() -> Unit
): Command{
    val builder = CommandBuilder().apply(block)
    var command: Command = if(logIndividualCommands){
        loggedSequentialCommandGroup(
            name,
            *builder.commands.toTypedArray()
        )
    }else{
        SequentialCommandGroup(
            *builder.commands.toTypedArray()
        ).withName(name)
    }.finallyDo(builder.endBehavior)

    if (builder.requirements.size > 0){
        command = command.withExtraRequirements(*builder.requirements.toTypedArray())
    }
    return command
}



@DslMarker
public annotation class CommandBuilderMarker

@CommandBuilderMarker
public class CommandBuilder {
    public var commands: LinkedHashSet<Command> = linkedSetOf() // LinkedHashSet keeps commands in order, but also ensures they're not added multiple times

    public val requirements: MutableList<Subsystem> = mutableListOf()

    public var endBehavior: (Boolean) -> Unit = {}
    
    /**
     * Adds a single command to be run until its completion.
     */
    public operator fun <C : Command> C.unaryPlus(): C{
        commands.add(this)
        return this
    }


    public fun addRequirements(vararg requirements: Subsystem){
        this.requirements.addAll(requirements)
    }

    /**
     * Runs the function block when the [buildCommand] is finished.
     */
    public fun onEnd(run: (Boolean) -> Unit){
        endBehavior = run
    }

    /**
     * Runs the function block when the [buildCommand] is finished.
     */
    @LowPriorityInOverloadResolution
    public inline fun onEnd(crossinline run: () -> Unit){
        endBehavior = { run() }
    }

    /**
     * Returns a command, then removes it from the set of commands within the [CommandBuilder].
     */
    public fun Command.withoutAdd(): Command = this.also(commands::remove)


    /**
     * Adds a command that will run once and then complete.
     *
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run
     */
    public fun runOnce(vararg requirements: Subsystem, execute: () -> Unit): InstantCommand =
        InstantCommand(*requirements) { execute() }.also(commands::add)


    /**
     * Adds a command that will run the command onTrue or only if a condition is met.
     */
    public fun runIf(condition: () -> Boolean, onTrue: Command, onFalse: Command): ConditionalCommand =
        ConditionalCommand(onTrue,onFalse,condition).also(commands::add)

    /**
     * Adds a command that will run the appropriate mapped command, depending on the key given.
     *
     * Commands that are specified within the [MappableContext] are automatically removed from the command set of the [CommandBuilder].
     *
     * @param key: A lambda that gets a generic value, used for choosing an appropriate command.
     * @param default: The default command called if the key does not match anything given within the [MappableContext] block.
     * @param block: The [MappableContext] that maps the key's value to a specific command.
     */
    public fun <T: Any> runWhen(key: () -> T, default: Command, block: MappableContext<T, Command>.() -> Unit): Command =
        CustomSelectCommand(
            key,
            MappableContext<T,Command>().apply(block).map.onEach { (_: T, command: Command) ->
                commands.remove(command)
            },
            default
        ).also(commands::add)


    /**
     * Runs a specific command out of many options when the correct index is passed in.
     */
    public fun runDependingOnIndex(
        indexSupplier: () -> Int,
        vararg commands: Command
    ): Command{
        var index = 0
        return CustomSelectCommand(
            indexSupplier,
            commandMap = commands.associateBy { index.also{index++} },
            default = InstantCommand{throw IndexOutOfBoundsException("The command index of your command is out of range.")}
        ).also(this.commands::add)
    }


    /**
     * Adds a command that will run *until* [condition] is met.
     *
     * @param condition the condition to be met
     * @param command the command to run until [condition] is met
     */
    public fun runUntil(condition: () -> Boolean, command: Command): ParallelRaceGroup =
        command.until { condition() }
            .also(commands::add)

    /**
     * Adds a command that will run *until* [condition] is met.
     *
     * @param condition the condition to be met
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run until [condition] is met
     */
    public inline fun loopUntil(noinline condition: () -> Boolean, vararg requirements: Subsystem, crossinline execute: () -> Unit): ParallelRaceGroup =
        runUntil(condition, RunCommand(*requirements) { execute() })

    /**
     * Adds a command that will run *while* [condition] is true.
     *
     * @param condition the condition to be met
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run
     */
    public inline fun loopWhile(crossinline condition: () -> Boolean, vararg requirements: Subsystem, noinline execute: () -> Unit): ParallelRaceGroup =
        loopUntil({ !condition() }, *requirements, execute=execute)

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
    public fun loopFor(timeInterval: Time, command: Command): ParallelRaceGroup {
        return command
            .withTimeout(timeInterval.inUnit(seconds))
            .also(commands::add)
    }

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param timeInterval the maximum allowed runtime of the command
     * @param requirements the Subsystems this command requires
     * @param execute the code to be run
     */
    public inline fun loopFor(timeInterval: Time, vararg requirements: Subsystem, crossinline execute: () -> Unit): ParallelRaceGroup =
        loopFor(timeInterval, RunCommand(*requirements) { execute() })

    /**
     * Adds a command to be run continuously.
     *
     * @param requirements the Subsystems this command requires
     * @param execute the code to be run
     */
    public fun loopForever(vararg requirements: Subsystem, execute: () -> Unit): RunCommand =
            RunCommand(*requirements) { execute() }
                .also(commands::add)

    /**
     * Adds a command that does nothing for a specified [timeInterval], then completes.
     *
     * Useful if a delay is needed between two commands in a [SequentialCommandGroup].
     * Note that running this in parallel with other commands is unlikely to be useful.
     */
    public fun waitFor(timeInterval: Time): ParallelRaceGroup = loopFor(timeInterval) {}

    /**
     * Adds a command that does nothing until a [condition] is met, then completes.
     *
     * Useful if some condition must be met before proceeding to the next command in a [SequentialCommandGroup].
     * Note that running this in parallel with other commands is unlikely to be useful.
     */
    public fun waitUntil(condition: () -> Boolean): ParallelRaceGroup = loopUntil(condition) {}

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
    public fun <T : Any> getOnceDuringRun(get: () -> T) : ReadOnlyProperty<Any?, T> =
        DuringRunGetter(get)

    private inner class DuringRunGetter<T : Any>(private val get: () -> T) : ReadOnlyProperty<Any?, T> {
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
            value = get()
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

    private inner class CustomSelectCommand<T: Any>(
        val getValue: () -> T,
        val commandMap: Map<T,Command>,
        default: Command
    ): CommandBase(){
        var allRequirements: Array<Subsystem> = arrayOf()
        var runsWhenDisabled = true

        var selectedCommand = default

        var interruptBehavior = InterruptionBehavior.kCancelIncoming

        init{
            for (command in commandMap.values){
                allRequirements += command.requirements.toTypedArray()
                runsWhenDisabled = runsWhenDisabled && command.runsWhenDisabled()
                if (command.interruptionBehavior == InterruptionBehavior.kCancelSelf) {
                    interruptBehavior = InterruptionBehavior.kCancelSelf
                }
            }
            CommandScheduler.getInstance().registerComposedCommands(
                *(a[default] + commandMap.values.toTypedArray())
            )
            addRequirements(
                *allRequirements
            )
        }

        override fun initialize(){
            val selector = getValue()

            commandMap[selector]?.let{
                selectedCommand = it
            }

            selectedCommand.initialize()
        }

        override fun execute() = selectedCommand.execute()
        override fun isFinished() = selectedCommand.isFinished
        override fun end(interrupted: Boolean) = selectedCommand.end(interrupted)
        override fun runsWhenDisabled() = runsWhenDisabled
        override fun getInterruptionBehavior() = interruptBehavior

    }
}




/**
 * Utility functions used for logging commands within the buildCommand DSL.
 */

@PublishedApi
internal fun loggedSequentialCommandGroup(name: String, vararg commands: Command): SequentialCommandGroup{
    val loggedCommands: Array<Command> = commands.map{it.withLogInCommandGroup(name)}.toTypedArray()
    return SequentialCommandGroup(
        *loggedCommands
    ).also{
        it.name = name
    }
}

internal fun Command.withLogInCommandGroup(commandGroupName: String): Command{


    fun logCommand(active: Boolean) = InstantCommand{
        Logger.getInstance().recordOutput(
            "/ActiveCommands/Subcommands Of: $commandGroupName/$name",active
        )
    }

    // uses custom infix "then" operator(more concise way to do andThen)
    return logCommand(true) then this then logCommand(false)
}

public object CodeBlockContext