package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

/**
 * A custom wrapper over AdvantageKit's [LoggableInputs],
 * which implements the toLog and fromLog functions using property delegates,
 * and supports kmeasure units.
 *
 * All Input classes for subsystems require fromLog and toLog functions
 * in order to support replay(which calls fromLog to override values that would have been sim-generated values).
 *
 * Typical LoggedInputs integration and usage:
 *
 * ```
 * public class DemoInputs: LoggableInputs {
 *     public var propertyOne: Double = 0.0
 *     public var propertyTwo: Angle = 0.0.degrees
 *     override fun toLog(table: LogTable?) {
 *         table?.put("propertyOne", propertyOne)
 *         table?.put("propertyTwoDeg", propertyTwo.inUnit(degrees))
 *     }
 *
 *     override fun fromLog(table: LogTable?) {
 *         propertyOne = table?.getDouble("propertyOne", 0.0) ?: 0.0
 *         propertyTwo = (table?.getDouble("propertyTwo", 0.0) ?: 0.0).ofUnit(degrees)
 *     }
 * }
 * ```
 *
 * ChargerLoggableInputs usage(fromLog and toLog are implemented
 * by the ChargerLoggableInputs parent class):
 *
 * ```
 * public class DemoInputs: ChargerLoggableInputs(){
 *      public var propertyOne by loggedDouble(0.0,"propertyOne")
 *      public var propertyTwo by loggedQuantity(0.0.degrees,"propertyTwo",logUnit = degrees)
 * }
 * ```
 *
 *
 *
 */
public abstract class ChargerLoggableInputs: LoggableInputs {



    /**
     * A property delegate that acts identically to a regular Double,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun loggedDouble(
        logName: String,
        defaultValue: Double = 0.0
    ): ReadWriteProperty<Any?,Double> =
        object: ReadWriteProperty<Any?,Double>{
            private var innerValue = defaultValue
            init{
                errorIfLogNameTaken(logName)
                doubleLoggers.add(
                    DoubleLogger(
                        logName = logName,
                        getDoubleValue = {innerValue},
                        setDoubleValue = {innerValue = it}
                    )
                )
            }
            override fun getValue(thisRef: Any?, property: KProperty<*>): Double = innerValue
            override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
                innerValue = value
            }
        }

    /**
     * A property delegate that acts identically to a regular nullable Double,
     * but also allows the fromLog and toLog functions of [LoggableInputs] to modify its value,
     * in addition to adding an "isValid" log category checking if the value is null or not.
     */
    public fun loggedNullableDouble(
        logName: String,
        defaultValue: Double? = null
    ): ReadWriteProperty<Any?,Double?> = object: ReadWriteProperty<Any?,Double?>{
        private var innerValue = defaultValue

        init{
            errorIfLogNameTaken(logName)
            doubleLoggers.add(
                DoubleLogger(
                    logName = logName,
                    getDoubleValue = { innerValue ?: 0.0 },
                    setDoubleValue = {innerValue = it}
                )
            )

            boolLoggers.add(
                BoolLogger(
                    logName = logName + "IsValid",
                    getBoolValue = {innerValue != null},
                    setBoolValue = { if (!it) {innerValue = null} }
                )
            )
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Double? = defaultValue

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double?) {
            this.innerValue = value
        }

    }

    /**
     * A property delegate that acts identically to a regular variable, storing a Quantity,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun <D: AnyDimension> loggedQuantity(
        logUnit: Quantity<D>,
        logName: String,
        defaultValue: Quantity<D> = Quantity(0.0)
    ): ReadWriteProperty<Any?, Quantity<D>> =
        object: ReadWriteProperty<Any?,Quantity<D>>{
            private var innerValue = defaultValue
            init{
                errorIfLogNameTaken(logName)
                doubleLoggers.add(
                    DoubleLogger(
                        logName = logName,
                        getDoubleValue = {innerValue.inUnit(logUnit)},
                        setDoubleValue = {innerValue = it.ofUnit(logUnit)}
                    )
                )
            }
            override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = innerValue
            override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>) {
                innerValue = value
            }
        }

    public fun <D: AnyDimension> loggedNullableQuantity(
        logUnit: Quantity<D>,
        logName: String,
        defaultValue: Quantity<D>? = null
    ): ReadWriteProperty<Any?, Quantity<D>?> =
        object: ReadWriteProperty<Any?,Quantity<D>?>{
            private var innerValue = defaultValue
            init{
                errorIfLogNameTaken(logName)
                doubleLoggers.add(
                    DoubleLogger(
                        logName = logName,
                        getDoubleValue = {innerValue?.inUnit(logUnit) ?: 0.0},
                        setDoubleValue = {innerValue = it.ofUnit(logUnit)}
                    )
                )

                boolLoggers.add(
                    BoolLogger(
                        logName = logName + "IsValid",
                        getBoolValue = {innerValue != null},
                        setBoolValue = {if (!it) {innerValue = null}}
                    )
                )
            }
            override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D>? = innerValue
            override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>?) {
                innerValue = value
            }
        }

    /**
     * A property delegate that acts identically to a regular variable, storing a Boolean,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun loggedBoolean(
        logName: String,
        defaultValue: Boolean = false
    ): ReadWriteProperty<Any?,Boolean> =
        object: ReadWriteProperty<Any?,Boolean>{
            private var innerValue = defaultValue

            init{
                errorIfLogNameTaken(logName)
                boolLoggers.add(
                    BoolLogger(
                        logName = logName,
                        getBoolValue = {innerValue},
                        setBoolValue = {innerValue = it}
                    )
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = innerValue

            override fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
                innerValue = value
            }
        }

    /**
     * A property delegate that acts identically to a regular variable, storing a String,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun loggedString(
        logName: String,
        defaultValue: String = "NO_VALUE"
    ): ReadWriteProperty<Any?,String> =
        object: ReadWriteProperty<Any?,String>{
            private var innerValue = defaultValue

            init{
                errorIfLogNameTaken(logName)
                stringLoggers.add(
                    StringLogger(
                        logName = logName,
                        getStringValue = {innerValue},
                        setStringValue = {innerValue = it}
                    )
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): String = innerValue

            override fun setValue(thisRef: Any?, property: KProperty<*>, value: String) {
                innerValue = value
            }
        }







    /**
     * A class used to allow the LoggableInputs class
     * set and get variables delegated with (something) from the toLog and fromLog functions,
     * which can get and set a double.
     */
    private class DoubleLogger(
        val logName: String,
        val getDoubleValue: () -> Double,
        val setDoubleValue: (Double) -> Unit
    )

    /**
     * A class used to allow the LoggableInputs class
     * set and get variables delegated with (something) from the toLog and fromLog functions,
     * which can get and set a boolean.
     */
    private class BoolLogger(
        val logName: String,
        val getBoolValue: () -> Boolean,
        val setBoolValue: (Boolean) -> Unit
    )

    /**
     * A class used to allow the LoggableInputs class
     * set and get variables delegated with (something) from the toLog and fromLog functions,
     * which can get and set a String.
     */
    private class StringLogger(
        val logName: String,
        val getStringValue: () -> String,
        val setStringValue: (String) -> Unit
    )


    // checks to see that there aren't 2 variables with the same log name
    private fun errorIfLogNameTaken(logName: String){
        if (logName in
            doubleLoggers.map{it.logName} +
            boolLoggers.map{it.logName} +
            stringLoggers.map{it.logName}
            ){
            error("Looks like one of the logName's you assigned to a variable is already taken by something else. Please change it.")
        }
    }
    private val doubleLoggers: MutableList<DoubleLogger> = mutableListOf()
    private val boolLoggers: MutableList<BoolLogger> = mutableListOf()
    private val stringLoggers: MutableList<StringLogger> = mutableListOf()

    /*
    Below are the fromLog and toLog implementations.

    table?.apply{} checks if the table is null; if it isn't, it logs the values.

    The put, getDouble, getString and getBoolean functions are part of the table itself;
    apply gives the function block below the context of the table itself
    (like calling functions within the table class).
     */


    override fun toLog(table: LogTable?) {
        table?.apply{
            doubleLoggers.forEach{
                put(it.logName,it.getDoubleValue())
            }
            boolLoggers.forEach{
                put(it.logName,it.getBoolValue())
            }
            stringLoggers.forEach{
                put(it.logName,it.getStringValue())
            }
        }
    }

    override fun fromLog(table: LogTable?) {
        table?.apply{
            doubleLoggers.forEach{
                it.setDoubleValue(
                    getDouble(
                        it.logName,
                        it.getDoubleValue()
                    )
                )

            }
            boolLoggers.forEach{
                it.setBoolValue(
                    getBoolean(
                        it.logName,
                        it.getBoolValue()
                    )
                )
            }

            stringLoggers.forEach{
                getString(it.logName,it.getStringValue())?.let{value: String ->
                    it.setStringValue(value)
                }
            }
        }


    }
}