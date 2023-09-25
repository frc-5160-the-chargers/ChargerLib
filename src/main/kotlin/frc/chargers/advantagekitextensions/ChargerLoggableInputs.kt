package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

public open class ChargerLoggableInputs: LoggableInputs {
    /**
     * A property delegate that acts identically to a regular variable,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun loggedDouble(value: Double, logName: String): ReadWriteProperty<Any?,Double> =
        LoggableDouble(value,logName)

    /**
     * A property delegate that acts identically to a regular variable, storing a Quantity,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    public fun <D: AnyDimension> loggedQuantity(value: Quantity<D>, logName: String, logUnit: Quantity<D>): ReadWriteProperty<Any?,Quantity<D>> =
        LoggableQuantity(value,logUnit,logName)







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


    private val doubleLoggers: MutableList<DoubleLogger> = mutableListOf()

    private inner class LoggableDouble(
        private var value: Double, logName: String
    ): ReadWriteProperty<Any?,Double>{
        init{
            doubleLoggers.add(
                DoubleLogger(
                    logName = logName,
                    getDoubleValue = {value},
                    setDoubleValue = {value = it}
                )
            )
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = value
        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            this.value = value
        }
    }

    private inner class LoggableQuantity<D: AnyDimension>(
        private var value: Quantity<D>,
        private val logUnit: Quantity<D>,
        logName: String
    ): ReadWriteProperty<Any?,Quantity<D>>{
        init{
            doubleLoggers.add(
                DoubleLogger(
                    logName = logName,
                    getDoubleValue = {value.inUnit(logUnit)},
                    setDoubleValue = {value = it.ofUnit(logUnit)}
                )
            )
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = value
        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>) {
            this.value = value
        }

    }
    override fun toLog(table: LogTable?) {
        doubleLoggers.forEach{
            table?.put(it.logName,it.getDoubleValue())
        }
    }

    override fun fromLog(table: LogTable?) {
        doubleLoggers.forEach{
            table?.getDouble(it.logName,it.getDoubleValue())?.let{value: Double ->
                it.setDoubleValue(value)
            }
        }
    }
}