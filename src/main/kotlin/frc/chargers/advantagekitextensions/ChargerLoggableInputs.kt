package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import frc.chargers.utils.math.units.KmeasureUnit
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

public typealias LoggableInput<T> = PropertyDelegateProvider<Any?,ReadWriteProperty<Any?,T>>


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
 *      public var propertyOne by loggedDouble()
 *      public var propertyTwo by loggedQuantity(logUnit = degrees, logName = "propertyTwoDeg")
 *      // log name is: propertyThree(SI Unit), logUnit is SI unit(radians)
 *      public var propertyThree: Angle by loggedQuantity()
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
    protected fun loggedDouble(
        logName: String? = null,
        default: Double = 0.0
    ): LoggableInput<Double> =
        PropertyDelegateProvider{ _, variable -> LoggedDoubleDelegate(logName ?: variable.name, default) }


    private inner class LoggedDoubleDelegate(
        logName: String,
        private var innerValue: Double = 0.0
    ): ReadWriteProperty<Any?,Double>{
        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = innerValue
        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            this.innerValue = value
        }

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

    }

    /**
     * A property delegate that acts identically to a regular nullable Double,
     * but also allows the fromLog and toLog functions of [LoggableInputs] to modify its value,
     * in addition to adding an "isValid" log category checking if the value is null or not.
     */
    protected fun loggedNullableDouble(
        name: String? = null,
        defaultValue: Double? = 0.0
    ): LoggableInput<Double?> =
        PropertyDelegateProvider{ _, variable -> LoggedNullableDoubleDelegate(name ?: variable.name,defaultValue) }

    private inner class LoggedNullableDoubleDelegate(
        logName: String,
        private var innerValue: Double? = 0.0
    ): ReadWriteProperty<Any?,Double?>{


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
        override fun getValue(thisRef: Any?, property: KProperty<*>): Double? = innerValue

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double?) {
            this.innerValue = value
        }
    }




    /**
     * A property delegate that acts identically to a regular variable, storing a Quantity,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    protected fun <D: AnyDimension> loggedQuantity(
        logUnit: Quantity<D> = Quantity(1.0),
        logName: String? = null,
        defaultValue: Quantity<D> = Quantity(0.0)
    ): LoggableInput<Quantity<D>> =
        PropertyDelegateProvider { _, variable ->
            val defaultLogName = logName ?: (variable.name + if(logUnit.siValue == 1.0) "(SI value)" else "" )
            LoggedQuantityDelegate(
                logUnit = logUnit,
                logName =  defaultLogName,
                defaultValue
            )
        }

    private inner class LoggedQuantityDelegate<D: AnyDimension>(
        logUnit: Quantity<D>,
        logName: String,
        private var innerValue: Quantity<D> = Quantity(0.0)
    ): ReadWriteProperty<Any?,Quantity<D>>{

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


    /**
     * A property delegate that acts identically to a regular variable, storing a nullable Quantity,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    protected fun <D: AnyDimension> loggedNullableQuantity(
        logUnit: Quantity<D> = Quantity(1.0),
        logName: String? = null,
        defaultValue: Quantity<D> = Quantity(0.0)
    ): LoggableInput<Quantity<D>?> =
        PropertyDelegateProvider { _, variable ->
            val defaultLogName = logName ?: (variable.name + if(logUnit.siValue == 1.0) "(SI value)" else "" )
            LoggedNullableQuantityDelegate(
                logUnit,
                defaultLogName,
                defaultValue
            )
        }

    private inner class LoggedNullableQuantityDelegate<D: AnyDimension>(
        logUnit: Quantity<D>,
        logName: String,
        private var innerValue: Quantity<D>? = Quantity(0.0)
    ): ReadWriteProperty<Any?,Quantity<D>?>{
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
     * A property delegate that acts identically to a regular variable, storing a String,
     * but also exposes its get and set functions to a MutableList,
     * which allows the fromLog and toLog functions of [LoggableInputs] to modify its value.
     */
    protected fun loggedBoolean(logName: String? = null, defaultValue: Boolean = false): LoggableInput<Boolean> =
        PropertyDelegateProvider{ _, variable -> LoggedBooleanDelegate(logName ?: variable.name ,defaultValue) }



    private inner class LoggedBooleanDelegate(
        logName: String,
        private var innerValue: Boolean
    ): ReadWriteProperty<Any?,Boolean>{
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
     * Represents a generic logged field.
     */
    protected fun loggedString(
        logName: String? = null,
        defaultValue: String = "Nothing"
    ): LoggableInput<String> =
        PropertyDelegateProvider{ _, variable ->
            LoggedStringDelegate(logName ?: variable.name, defaultValue)
        }




    private inner class LoggedStringDelegate(
        logName: String,
        private var innerValue: String
    ): ReadWriteProperty<Any?, String>{
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





    protected fun loggedDoubles(
        logName: String? = null,
        defaultValue: List<Double> = listOf()
    ): LoggableInput<List<Double>> =
        PropertyDelegateProvider { _, variable ->
            LoggedDoubleListDelegate(logName ?: variable.name, defaultValue){it}
        }

    protected fun loggedMutableDoubles(
        logName: String? = null,
        defaultValue: MutableList<Double> = mutableListOf()
    ): LoggableInput<MutableList<Double>> =
        PropertyDelegateProvider{_, variable ->
            LoggedDoubleListDelegate(logName ?: variable.name, defaultValue){ it.toMutableList() }
        }


    private inner class LoggedDoubleListDelegate <T: List<Double>>(
        logName: String,
        private var innerValue: T,
        private val convertListToT: (List<Double>) -> T
    ): ReadWriteProperty<Any?,T>{

        init{
            errorIfLogNameTaken(logName)
            doubleListLoggers.add(
                DoubleListLogger(
                    logName = logName,
                    getValue = {innerValue},
                    setValue = {innerValue = convertListToT(it)}
                )
            )
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): T = innerValue
        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
            innerValue = value
        }

    }


    protected fun <D: AnyDimension> loggedQuantities(
        logUnit: KmeasureUnit<D> = KmeasureUnit(1.0),
        logName: String? = null,
        defaultValue: List<Quantity<D>> = listOf()
    ): LoggableInput<List<Quantity<D>>> =
        PropertyDelegateProvider { _, variable ->
            val defaultName = variable.name + if(logUnit == KmeasureUnit<D>(1.0)) "(Si Unit)" else ""
            LoggedQuantityListDelegate(logUnit, logName ?: defaultName, defaultValue){it}
        }

    protected fun <D: AnyDimension> loggedMutableQuantities(
        logUnit: KmeasureUnit<D> = KmeasureUnit(1.0),
        logName: String? = null,
        defaultValue: MutableList<Quantity<D>> = mutableListOf()
    ): LoggableInput<MutableList<Quantity<D>>> =
        PropertyDelegateProvider { _, variable ->
            val defaultName = variable.name + if(logUnit == KmeasureUnit<D>(1.0)) "(Si Unit)" else ""
            LoggedQuantityListDelegate(logUnit, logName ?: defaultName, defaultValue){it.toMutableList()}
        }


    private inner class LoggedQuantityListDelegate <D: AnyDimension, T: List<Quantity<D>>>(
        logUnit: KmeasureUnit<D>,
        logName: String,
        private var innerValue: T,
        private val convertListToT: (List<Quantity<D>>) -> T
    ): ReadWriteProperty<Any?,T>{

        init{
            errorIfLogNameTaken(logName)
            doubleListLoggers.add(
                DoubleListLogger(
                    logName = logName,
                    getValue = {innerValue.map {it.inUnit(logUnit)} },
                    setValue = { innerValue = convertListToT( it.map { value -> value.ofUnit(logUnit) } ) }
                )
            )
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): T = innerValue
        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
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


    private class DoubleListLogger(
        val logName: String,
        val getValue: () -> List<Double>,
        val setValue: (List<Double>) -> Unit
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
    private val doubleListLoggers: MutableList<DoubleListLogger> = mutableListOf()

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

            doubleListLoggers.forEach{
                put(it.logName,it.getValue().toDoubleArray())
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

            doubleListLoggers.forEach{
                getDoubleArray(it.logName,it.getValue().toDoubleArray())?.let{value: DoubleArray ->
                    it.setValue(value.toList())
                }
            }

        }


    }
}

