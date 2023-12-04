package frc.chargers.advantagekitextensions


import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.Measurement
import frc.chargers.utils.NullableMeasurement
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.internal.LowPriorityInOverloadResolution
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

/**
 * Represents a Loggable input property delegate that is read only.
 */
public typealias ReadOnlyLoggableInput<T> = PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?,T>>
/**
 * Represents a Loggable input property delegate that has read-write properties.
 */
public typealias ReadWriteLoggableInput<T> = PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>>
/**
 * Represents a generic scope where a field can be accessed;
 * mimicking the field term in kotlin's setters.
 */
public interface AccessorScope<D>{
    public var field: D
}
/**
 * Represents a generic function that can set the value of a class.
 */
public typealias Setter<T> = AccessorScope<T>.(T) -> Unit



public class LoggableInputsProvider(
    public val logGroup: String
){
    public fun int(getValue: () -> Int): ReadOnlyLoggableInput<Int> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedInt(variable.name, getValue) }
    public fun double(getValue: () -> Double): ReadOnlyLoggableInput<Double> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDouble(variable.name, getValue) }
    public fun <D: AnyDimension> quantity(getValue: () -> Quantity<D>): ReadOnlyLoggableInput<Quantity<D>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantity(variable.name + "(SI value)", getValue) }
    public fun boolean(getValue: () -> Boolean): ReadOnlyLoggableInput<Boolean> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBoolean(variable.name, getValue) }
    public fun string(getValue: () -> String): ReadOnlyLoggableInput<String> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedString(variable.name, getValue) }



    public fun nullableInt(getValue: () -> Int?): ReadOnlyLoggableInput<Int?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableInt(variable.name, getValue) }
    public fun nullableDouble(getValue: () -> Double?): ReadOnlyLoggableInput<Double?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableDouble(variable.name, getValue) }
    public fun <D: AnyDimension> nullableQuantity(getValue: () -> Quantity<D>?): ReadOnlyLoggableInput<Quantity<D>?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableQuantity(variable.name + "(SI value)", getValue) }



    public fun intList(getValue: () -> List<Int>): ReadOnlyLoggableInput<List<Int>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedIntList(variable.name, getValue) }
    public fun <D: AnyDimension> quantityList(getValue: () -> List<Quantity<D>>): ReadOnlyLoggableInput<List<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantityList(variable.name + "(SI Value)", getValue) }
    public fun doubleList(getValue: () -> List<Double>): ReadOnlyLoggableInput<List<Double>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDoubleList(variable.name, getValue) }
    public fun booleanList(getValue: () -> List<Boolean>): ReadOnlyLoggableInput<List<Boolean>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBooleanList(variable.name, getValue) }
    public fun stringList(getValue: () -> List<String>): ReadOnlyLoggableInput<List<String>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedStringList(variable.name, getValue) }


    public fun <T: AdvantageKitLoggable<T>> genericValue(getValue: () -> T): ReadOnlyLoggableInput<T> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericValue(variable.name,getValue)}
    public fun <T: AdvantageKitLoggable<T>> genericNullableValue(loggedNullRepr: T, getValue: () -> T?): ReadOnlyLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericNullableValue(variable.name,loggedNullRepr, getValue)}
    public fun <D: AnyDimension> timestampedQuantity(getValue: () -> Measurement<Quantity<D>>): ReadOnlyLoggableInput<Measurement<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantityMeasurement(variable.name, getValue)}
    public fun <D: AnyDimension> timestampedNullableQuantity(getValue: () -> NullableMeasurement<Quantity<D>>): ReadOnlyLoggableInput<NullableMeasurement<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableQuantityMeasurement(variable.name, getValue)}
    public fun <T: AdvantageKitLoggable<T>> timestampedValue(getValue: () -> Measurement<T>): ReadOnlyLoggableInput<Measurement<T>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedMeasurement(variable.name, getValue)}
    public fun <T: AdvantageKitLoggable<T>> timestampedNullableValue(logNullRepr: T, getValue: () -> NullableMeasurement<T>): ReadOnlyLoggableInput<NullableMeasurement<T>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableMeasurement(variable.name, logNullRepr, getValue)}





    @LowPriorityInOverloadResolution
    public fun int(getValue: () -> Int, setValue: Setter<Int> = {field = it} ): ReadWriteLoggableInput<Int> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedInt(variable.name, getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun double(getValue: () -> Double, setValue: Setter<Double> = {field = it}): ReadWriteLoggableInput<Double> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDouble(variable.name, getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun <D: AnyDimension> quantity(getValue: () -> Quantity<D>, setValue: Setter<Quantity<D>> = {field = it}): ReadWriteLoggableInput<Quantity<D>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantity(variable.name + "(SI value)", getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun boolean(getValue: () -> Boolean, setValue: Setter<Boolean> = {field = it}): ReadWriteLoggableInput<Boolean> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBoolean(variable.name, getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun string(getValue: () -> String, setValue: Setter<String> = {field = it}): ReadWriteLoggableInput<String> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedString(variable.name, getValue,setValue) }



    @LowPriorityInOverloadResolution
    public fun nullableInt(getValue: () -> Int?, setValue: Setter<Int?> = {field = it}): ReadWriteLoggableInput<Int?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableInt(variable.name, getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun nullableDouble(getValue: () -> Double?, setValue: Setter<Double?> = {field = it}): ReadWriteLoggableInput<Double?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableDouble(variable.name, getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun <D: AnyDimension> nullableQuantity(getValue: () -> Quantity<D>?, setValue: Setter<Quantity<D>?> = {field = it}): ReadWriteLoggableInput<Quantity<D>?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableQuantity(variable.name + "(SI value)", getValue,setValue) }


    @LowPriorityInOverloadResolution
    public fun intList(getValue: () -> List<Int>, setValue: Setter<List<Int>> = {field = it}): ReadWriteLoggableInput<List<Int>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedIntList(variable.name, getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun <D: AnyDimension> quantityList(
        getValue: () -> List<Quantity<D>>,
        setValue: Setter<List<Quantity<D>>> = {field = it}
    ): ReadWriteLoggableInput<List<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantityList(variable.name + "(SI Value)", getValue,setValue) }
    @LowPriorityInOverloadResolution
    public fun doubleList(
        getValue: () -> List<Double>,
        setValue: Setter<List<Double>> = {field = it}
    ): ReadWriteLoggableInput<List<Double>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDoubleList(variable.name, getValue, setValue) }
    @LowPriorityInOverloadResolution
    public fun booleanList(
        getValue: () -> List<Boolean>,
        setValue: Setter<List<Boolean>> = {field = it}
    ): ReadWriteLoggableInput<List<Boolean>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBooleanList(variable.name, getValue, setValue) }
    @LowPriorityInOverloadResolution
    public fun stringList(getValue: () -> List<String>, setValue: Setter<List<String>> = {field = it}): ReadWriteLoggableInput<List<String>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedStringList(variable.name, getValue, setValue) }

    @LowPriorityInOverloadResolution
    public fun <T: AdvantageKitLoggable<T>> genericValue(getValue: () -> T, setValue: Setter<T> = {field = it}): ReadWriteLoggableInput<T> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedGenericValue(variable.name,getValue,setValue)}

    @LowPriorityInOverloadResolution
    public fun <T: AdvantageKitLoggable<T>> genericNullableValue(nullReprWhenLogged: T, getValue: () -> T?, setValue: Setter<T?> = {field = it}): ReadWriteLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericNullableValue(variable.name,nullReprWhenLogged, getValue, setValue)}

    @LowPriorityInOverloadResolution
    public fun <D: AnyDimension> timestampedQuantity(getValue: () -> Measurement<Quantity<D>>, setValue: Setter<Measurement<Quantity<D>>> = {field = it}): ReadWriteLoggableInput<Measurement<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantityMeasurement(variable.name, getValue, setValue)}

    @LowPriorityInOverloadResolution
    public fun <D: AnyDimension> timestampedNullableQuantity(getValue: () -> NullableMeasurement<Quantity<D>>, setValue: Setter<NullableMeasurement<Quantity<D>>> = {field = it}): ReadWriteLoggableInput<NullableMeasurement<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableQuantityMeasurement(variable.name, getValue, setValue)}

    @LowPriorityInOverloadResolution
    public fun <T: AdvantageKitLoggable<T>> timestampedValue(getValue: () -> Measurement<T>, setValue: Setter<Measurement<T>> = {field = it}): ReadWriteLoggableInput<Measurement<T>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedMeasurement(variable.name, getValue,setValue)}

    @LowPriorityInOverloadResolution
    public fun <T: AdvantageKitLoggable<T>> timestampedNullableValue(nullReprWhenLogged: T, getValue: () -> NullableMeasurement<T>, setValue: Setter<NullableMeasurement<T>> = {field = it}): ReadWriteLoggableInput<NullableMeasurement<T>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableMeasurement(variable.name, nullReprWhenLogged, getValue,setValue)}


    private inner class AutoLoggedInt(
        val name: String,
        val get: () -> Int,
        val set: Setter<Int> = {field = it}
    ): ReadWriteProperty<Any?, Int>, AccessorScope<Int>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toLong())
            override fun fromLog(table: LogTable) { field = table.getInteger(name,0).toInt()}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int) {
            set(value)
        }

    }


    private inner class AutoLoggedDouble(
        val name: String,
        val get: () -> Double,
        val set: Setter<Double> = {field = it}
    ): ReadWriteProperty<Any?, Double>, AccessorScope<Double>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.getDouble(name,0.0) }
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            set(value)
        }
    }


    private inner class AutoLoggedQuantity<D: AnyDimension>(
        val name: String,
        val get: () -> Quantity<D>,
        val set: Setter<Quantity<D>> = {field = it}
    ): ReadWriteProperty<Any?, Quantity<D>>, AccessorScope<Quantity<D>>{
        override var field = get()


        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.siValue)
            override fun fromLog(table: LogTable) { field = Quantity(table.getDouble(name,0.0)) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>) {
            set(value)
        }
    }

    private inner class AutoLoggedBoolean(
        val name: String,
        val get: () -> Boolean,
        val set: Setter<Boolean> = {field = it}
    ): ReadWriteProperty<Any?,Boolean>, AccessorScope<Boolean>{
        override var field = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.getBoolean(name,false) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
            set(value)
        }

    }


    private inner class AutoLoggedString(
        val name: String,
        val get: () -> String,
        val set: Setter<String> = {field = it}
    ): ReadWriteProperty<Any?,String>, AccessorScope<String>{
        override var field = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.getString(name,"NOTHING") }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): String = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: String) {
            set(value)
        }
    }

    private inner class AutoLoggedNullableInt(
        val name: String,
        val get: () -> Int?,
        val set: Setter<Int?> = {field = it}
    ): ReadWriteProperty<Any?, Int?>, AccessorScope<Int?>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name,(field ?: 0).toLong())
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.getBoolean(name + "isValid", false)){
                    table.getInteger(name,0).toInt()
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int? = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int?) {
            set(value)
        }
    }



    private inner class AutoLoggedNullableDouble(
        val name: String,
        val get: () -> Double?,
        val set: Setter<Double?> = {field = it}
    ): ReadWriteProperty<Any?, Double?>, AccessorScope<Double?> {
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name,field ?: 0.0)
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.getBoolean(name + "isValid", false)){
                    table.getDouble(name,0.0)
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Double? = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double?) {
            set(value)
        }
    }



    private inner class AutoLoggedNullableQuantity<D: AnyDimension>(
        val name: String,
        val get: () -> Quantity<D>?,
        val set: Setter<Quantity<D>?> = {field = it}
    ): ReadWriteProperty<Any?, Quantity<D>?>, AccessorScope<Quantity<D>?>{
        override var field = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name, field?.siValue ?: 0.0)
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.getBoolean(name + "isValid",false)){
                    Quantity(table.getDouble(name,0.0))
                }else{
                    null
                }
            }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D>? = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>?) {
            set(value)
        }
    }



    private inner class AutoLoggedIntList(
        val name: String,
        val get: () -> List<Int>,
        val set: Setter<List<Int>> = {field = it}
    ): ReadWriteProperty<Any?,List<Int>>, AccessorScope<List<Int>>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.toLong()}.toLongArray())
            override fun fromLog(table: LogTable) { field = table.getIntegerArray(name,longArrayOf()).map{it.toInt()}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Int> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Int>) {
            set(value)
        }
    }


    private inner class AutoLoggedQuantityList<D: AnyDimension>(
        val name: String,
        val get: () -> List<Quantity<D>>,
        val set: Setter<List<Quantity<D>>> = {field = it}
    ): ReadWriteProperty<Any?,List<Quantity<D>>>, AccessorScope<List<Quantity<D>>>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.siValue}.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.getDoubleArray(name,doubleArrayOf()).map{Quantity(it)}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Quantity<D>> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Quantity<D>>) {
            set(value)
        }
    }


    private inner class AutoLoggedDoubleList(
        val name: String,
        val get: () -> List<Double>,
        val set: Setter<List<Double>> = {field = it}
    ): ReadWriteProperty<Any?,List<Double>>, AccessorScope<List<Double>>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.getDoubleArray(name, doubleArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Double> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Double>) {
            set(value)
        }
    }


    private inner class AutoLoggedBooleanList(
        val name: String,
        val get: () -> List<Boolean>,
        val set: Setter<List<Boolean>> = {field = it}
    ): ReadWriteProperty<Any?,List<Boolean>>, AccessorScope<List<Boolean>>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toBooleanArray())
            override fun fromLog(table: LogTable) { field = table.getBooleanArray(name, booleanArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Boolean> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Boolean>) {
            set(value)
        }
    }

    private inner class AutoLoggedStringList(
        val name: String,
        val get: () -> List<String>,
        val set: Setter<List<String>> = {field = it}
    ): ReadWriteProperty<Any?,List<String>>, AccessorScope<List<String>>{
        override var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toTypedArray())
            override fun fromLog(table: LogTable) { field = table.getStringArray(name,arrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<String> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<String>) {
            set(value)
        }
    }

    private inner class AutoLoggedGenericValue<T: AdvantageKitLoggable<T>>(
        val name: String,
        val get: () -> T,
        val set: Setter<T> = {field = it}
    ): ReadWriteProperty<Any?,T>, AccessorScope<T>{
        override var field: T = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) { field.pushToLog(table,name) }

            override fun fromLog(table: LogTable) { field = field.getFromLog(table,name) }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup,dummyInputs)
            }
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): T = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
            set(value)
        }
    }



    private inner class AutoLoggedGenericNullableValue<T: AdvantageKitLoggable<T>>(
        val name: String,
        val logNullRepr: T,
        val get: () -> T?,
        val set: Setter<T?> = {field = it}
    ): ReadWriteProperty<Any?,T?>, AccessorScope<T?>{
        override var field: T? = get()


        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup,dummyInputs)
            }
        }

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                val value = field
                if (value != null){
                    value.pushToLog(table,name)
                    table.put(name+"IsValid",true)
                }else{
                    logNullRepr.pushToLog(table,name)
                    table.put(name+"IsValid",true)
                }
            }

            override fun fromLog(table: LogTable) {
                val value: T = field ?: logNullRepr
                field = if (table.getBoolean(name+"IsValid",false)){
                    value.getFromLog(table,name)
                }else{
                    null
                }
            }
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): T? = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T?) {
            set(value)
        }
    }

    private inner class AutoLoggedNullableQuantityMeasurement<D: AnyDimension>(
        val name: String,
        val get: () -> NullableMeasurement<Quantity<D>>,
        val set: Setter<NullableMeasurement<Quantity<D>>> = {field = it}
    ): ReadWriteProperty<Any?,NullableMeasurement<Quantity<D>>>, AccessorScope<NullableMeasurement<Quantity<D>>>{
        override var field: NullableMeasurement<Quantity<D>> = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                table.apply{
                    put("$name/value(SI Unit)", field.nullableValue?.siValue ?: 0.0)
                    put("$name/isValid",field.nullableValue != null)
                    put("$name/timestampSecs",field.timestamp.inUnit(seconds))
                }
            }

            override fun fromLog(table: LogTable) {
                field = NullableMeasurement(
                    nullableValue = if (table.getBoolean("$name/isValid",false)){
                        Quantity(table.getDouble("$name/value(SI Unit)", 0.0))
                    }else{
                        null
                    },
                    timestamp = table.getDouble("$name/timestampSecs",0.0).ofUnit(seconds)
                )
            }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup,dummyInputs)
            }
        }



        override fun getValue(thisRef: Any?, property: KProperty<*>): NullableMeasurement<Quantity<D>> = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: NullableMeasurement<Quantity<D>>) {
            set(value)
        }

    }

    private inner class AutoLoggedQuantityMeasurement<D: AnyDimension>(
        val name: String,
        val get: () -> Measurement<Quantity<D>>,
        val set: Setter<Measurement<Quantity<D>>> = {field = it}
    ): ReadWriteProperty<Any?,Measurement<Quantity<D>>>, AccessorScope<Measurement<Quantity<D>>>{
        override var field: Measurement<Quantity<D>> = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                table.apply{
                    put("$name/value(SI Unit)", field.value.siValue)
                    put("$name/timestampSecs",field.timestamp.inUnit(seconds))
                }
            }

            override fun fromLog(table: LogTable) {
                field = Measurement(
                    value = Quantity(table.getDouble("$name/value(SI Unit)", 0.0)),
                    timestamp = table.getDouble("$name/timestampSecs",0.0).ofUnit(seconds)
                )
            }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup,dummyInputs)
            }
        }



        override fun getValue(thisRef: Any?, property: KProperty<*>): Measurement<Quantity<D>> = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Measurement<Quantity<D>>) {
            set(value)
        }

    }


    private inner class AutoLoggedNullableMeasurement<T: AdvantageKitLoggable<T>>(
        val name: String,
        val logNullRepr: T,
        val get: () -> NullableMeasurement<T>,
        val set: Setter<NullableMeasurement<T>> = {field = it}
    ): ReadWriteProperty<Any?,NullableMeasurement<T>>, AccessorScope<NullableMeasurement<T>>{
        override var field: NullableMeasurement<T> = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                table.apply{
                    field.nullableValue?.pushToLog(table, "$name/value") ?: logNullRepr.pushToLog(table,name)
                    put("$name/isValid",field.nullableValue != null)
                    put("$name/timestampSecs",field.timestamp.inUnit(seconds))
                }
            }

            override fun fromLog(table: LogTable) {
                field = NullableMeasurement(
                    nullableValue = if (table.getBoolean("$name/isValid",false)){
                        logNullRepr.getFromLog(table,"$name/value")
                    }else{
                        null
                    },
                    timestamp = table.getDouble("$name/timestampSecs",0.0).ofUnit(seconds)
                )
            }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup,dummyInputs)
            }
        }



        override fun getValue(thisRef: Any?, property: KProperty<*>): NullableMeasurement<T> = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: NullableMeasurement<T>) {
            set(value)
        }

    }


    private inner class AutoLoggedMeasurement<T: AdvantageKitLoggable<T>>(
        val name: String,
        val get: () -> Measurement<T>,
        val set: Setter<Measurement<T>> = {field = it}
    ): ReadWriteProperty<Any?,Measurement<T>>, AccessorScope<Measurement<T>>{
        override var field: Measurement<T> = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                table.apply{
                    field.value.pushToLog(table, "$name/value")
                    put("$name/timestampSecs",field.timestamp.inUnit(seconds))
                }
            }

            override fun fromLog(table: LogTable) {
                field = Measurement(
                    value = field.value.getFromLog(table,"$name/value"),
                    timestamp = table.getDouble("$name/timestampSecs",0.0).ofUnit(seconds)
                )
            }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.getInstance().processInputs(logGroup,dummyInputs)
            }
        }



        override fun getValue(thisRef: Any?, property: KProperty<*>): Measurement<T> = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Measurement<T>) {
            set(value)
        }

    }



    


}

