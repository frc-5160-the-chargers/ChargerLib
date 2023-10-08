package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

public abstract class ChargerLoggableInputs: LoggableInputs {
    /**
     * A property delegate that acts identically to a regular variable,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun loggedDouble(value: Double, logName: String): ReadWriteProperty<Any?,Double> =
        object: ReadWriteProperty<Any?,Double>{
            private var innerValue = value
            init{
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
     * A property delegate that acts identically to a regular variable, storing a Quantity,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun <D: AnyDimension> loggedQuantity(value: Quantity<D>, logName: String, logUnit: Quantity<D>): ReadWriteProperty<Any?,Quantity<D>> =
        object: ReadWriteProperty<Any?,Quantity<D>>{
            private var innerValue = value
            init{
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

    /**
     * A property delegate that acts identically to a regular variable, storing a Boolean,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun loggedBoolean(value: Boolean, logName: String): ReadWriteProperty<Any?,Boolean> =
        object: ReadWriteProperty<Any?,Boolean>{
            private var innerValue = value

            init{
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
    public fun loggedString(value: String, logName: String): ReadWriteProperty<Any?,String> =
        object: ReadWriteProperty<Any?,String>{
            private var innerValue = value

            init{
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


    private val doubleLoggers: MutableList<DoubleLogger> = mutableListOf()
    private val boolLoggers: MutableList<BoolLogger> = mutableListOf()
    private val stringLoggers: MutableList<StringLogger> = mutableListOf()



    override fun toLog(table: LogTable?) {
        doubleLoggers.forEach{
            table?.put(it.logName,it.getDoubleValue())
        }
        boolLoggers.forEach{
            table?.put(it.logName,it.getBoolValue())
        }
        stringLoggers.forEach{
            table?.put(it.logName,it.getStringValue())
        }
    }

    override fun fromLog(table: LogTable?) {
        doubleLoggers.forEach{
            table?.getDouble(it.logName,it.getDoubleValue())?.let{value: Double ->
                it.setDoubleValue(value)
            }
        }
        boolLoggers.forEach{
            table?.getBoolean(it.logName,it.getBoolValue())?.let{value: Boolean ->
                it.setBoolValue(value)
            }
        }

        stringLoggers.forEach{
            table?.getString(it.logName,it.getStringValue())?.let{value: String ->
                it.setStringValue(value)
            }
        }

    }
}