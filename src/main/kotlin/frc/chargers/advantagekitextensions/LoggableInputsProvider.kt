package frc.chargers.advantagekitextensions


import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import frc.chargers.framework.ChargerRobot
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
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
 * A wrapper around AdvantageKit which manages logging and replaying loggable inputs automatically,
 * without extra work.
 *
 * Usage example:
 *
 * ```
 * // have 1 LoggableInputsProvider per subsystem/log category
 * val ArmLog = LoggableInputsProvider("Arm")
 *
 * public interface ArmIO{
 *      public fun lowLevelFunction(input: Angle){}
 *
 *      // instead of a seperate "ArmIOInputs" class, low-level inputs are now part of
 *      // the io interface instead and are overriden by implementing classes
 *      public val current: Current
 *      public var appliedVoltage: Voltage
 *      public val otherProperty: Double
 * }
 *
 * public class ArmIOReal: ArmIO{
 *      // this value is now automatically logged under
 *      // "Arm/current(SI value)" and replayed from the same field.
 *      // no need for a processInputs or updateInputs function at all!
 *      override val current by ArmLog.quantity{...}
 *
 *      // kotlin custom getters and setters are also supported!
 *      override var appliedVoltage by ArmLog.quantity(
 *          getValue = {...},
 *          setValue = {...}
*       )
 *
 *      // logged under "Arm/otherProperty"
 *      // always use lowercase version of class name for delegates, for example:
 *      // Quantity<D> - LoggableInputsProvider.quantity{...}
 *      override val otherProperty by ArmLog.double{...}
 *      ...
 * }
 * ```
 *
 * Readability note: @PublishedApi internal makes a class or function
 * visible everywhere within the module and in inline functions,
 * but not in other places; this allows for performance increase through inlining.
 */
public class LoggableInputsProvider(
    public val namespace: String,
    public val shouldUpdateInputs: Boolean = true
){


    /**
     * Creates a subtab of the [LoggableInputsProvider].
     *
     * ```
     * val logInputsProviderOne = LoggableInputsProvider("Hello")
     *
     * // will now log under "Hello/goodbye"
     * val logInputsProviderTwo = logInputsProviderONe.subgroup("goodbye")
     *
     * ```
     */
    public fun subgroup(group: String): LoggableInputsProvider =
        LoggableInputsProvider("$namespace/$group")


    public inline fun int(
        crossinline getValue: () -> Int
    ): ReadOnlyLoggableInput<Int> =
        PropertyDelegateProvider{ _, variable -> loggedIntPrivateImpl(variable.name, getValue) }

    public inline fun double(
        crossinline getValue: () -> Double
    ): ReadOnlyLoggableInput<Double> =
        PropertyDelegateProvider{ _, variable -> loggedDoublePrivateImpl(variable.name, getValue) }

    public inline fun <D: AnyDimension> quantity(
        crossinline getValue: () -> Quantity<D>
    ): ReadOnlyLoggableInput<Quantity<D>> =
        PropertyDelegateProvider{ _, variable -> loggedQuantityPrivateImpl(variable.name + "(SI value)", getValue) }

    public inline fun boolean(
        crossinline getValue: () -> Boolean
    ): ReadOnlyLoggableInput<Boolean> =
        PropertyDelegateProvider{ _, variable -> loggedBooleanPrivateImpl(variable.name, getValue) }

    public inline fun string(
        crossinline getValue: () -> String
    ): ReadOnlyLoggableInput<String> =
        PropertyDelegateProvider{ _, variable -> loggedStringPrivateImpl(variable.name, getValue) }



    public inline fun nullableInt(
        crossinline getValue: () -> Int?
    ): ReadOnlyLoggableInput<Int?> =
        PropertyDelegateProvider{ _, variable -> loggedNullableIntPrivateImpl(variable.name, getValue) }

    public inline fun nullableDouble(
        crossinline getValue: () -> Double?
    ): ReadOnlyLoggableInput<Double?> =
        PropertyDelegateProvider{ _, variable -> loggedNullableDoublePrivateImpl(variable.name, getValue) }

    public inline fun <D: AnyDimension> nullableQuantity(
        crossinline getValue: () -> Quantity<D>?
    ): ReadOnlyLoggableInput<Quantity<D>?> =
        PropertyDelegateProvider{ _, variable -> loggedNullableQuantityPrivateImpl(variable.name + "(SI value)", getValue) }



    public inline fun intList(
        crossinline getValue: () -> List<Int>
    ): ReadOnlyLoggableInput<List<Int>> =
        PropertyDelegateProvider{ _, variable -> loggedIntListPrivateImpl(variable.name, getValue) }

    public inline fun <D: AnyDimension> quantityList(
        crossinline getValue: () -> List<Quantity<D>>
    ): ReadOnlyLoggableInput<List<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> loggedQuantityListPrivateImpl(variable.name + "(SI Value)", getValue) }

    public inline fun doubleList(
        crossinline getValue: () -> List<Double>
    ): ReadOnlyLoggableInput<List<Double>> =
        PropertyDelegateProvider{ _, variable -> loggedDoubleListPrivateImpl(variable.name, getValue) }

    public inline fun booleanList(
        crossinline getValue: () -> List<Boolean>
    ): ReadOnlyLoggableInput<List<Boolean>> =
        PropertyDelegateProvider{ _, variable -> loggedBooleanListPrivateImpl(variable.name, getValue) }

    public inline fun stringList(
        crossinline getValue: () -> List<String>
    ): ReadOnlyLoggableInput<List<String>> =
        PropertyDelegateProvider{ _, variable -> loggedStringListPrivateImpl(variable.name, getValue) }


    public inline fun <T: AdvantageKitLoggable<T>> value(
        default: T,
        crossinline getValue: () -> T
    ): ReadOnlyLoggableInput<T> =
        PropertyDelegateProvider{_, variable -> loggedGenericValuePrivateImpl(variable.name, default, getValue)}

    public inline fun <T: AdvantageKitLoggable<T>> nullableValue(
        default: T,
        crossinline getValue: () -> T?
    ): ReadOnlyLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> loggedGenericNullableValuePrivateImpl(variable.name,default, getValue)}






    
    public inline fun int(crossinline getValue: () -> Int, crossinline setValue: (Int) -> Unit): ReadWriteLoggableInput<Int> =
        PropertyDelegateProvider{ _, variable -> loggedIntPrivateImpl(variable.name, getValue,setValue) }

    public inline fun double(crossinline getValue: () -> Double, crossinline setValue: (Double) -> Unit): ReadWriteLoggableInput<Double> =
        PropertyDelegateProvider{ _, variable -> loggedDoublePrivateImpl(variable.name, getValue,setValue) }

    public inline fun <D: AnyDimension> quantity(
        crossinline getValue: () -> Quantity<D>,
        crossinline setValue: (Quantity<D>) -> Unit
    ): ReadWriteLoggableInput<Quantity<D>> =
        PropertyDelegateProvider{ _, variable -> loggedQuantityPrivateImpl(variable.name + "(SI value)", getValue,setValue) }

    public inline fun boolean(
        crossinline getValue: () -> Boolean,
        crossinline setValue: (Boolean) -> Unit
    ): ReadWriteLoggableInput<Boolean> =
        PropertyDelegateProvider{ _, variable -> loggedBooleanPrivateImpl(variable.name, getValue,setValue) }

    public inline fun string(
        crossinline getValue: () -> String,
        crossinline setValue: (String) -> Unit
    ): ReadWriteLoggableInput<String> =
        PropertyDelegateProvider{ _, variable -> loggedStringPrivateImpl(variable.name, getValue,setValue) }



    
    public inline fun nullableInt(
        crossinline getValue: () -> Int?,
        crossinline setValue: (Int?) -> Unit
    ): ReadWriteLoggableInput<Int?> =
        PropertyDelegateProvider{ _, variable -> loggedNullableIntPrivateImpl(variable.name, getValue,setValue) }

    public inline fun nullableDouble(
        crossinline getValue: () -> Double?,
        crossinline setValue: (Double?) -> Unit
    ): ReadWriteLoggableInput<Double?> =
        PropertyDelegateProvider{ _, variable -> loggedNullableDoublePrivateImpl(variable.name, getValue,setValue) }

    public inline fun <D: AnyDimension> nullableQuantity(
        crossinline getValue: () -> Quantity<D>?,
        crossinline setValue: (Quantity<D>?) -> Unit
    ): ReadWriteLoggableInput<Quantity<D>?> =
        PropertyDelegateProvider{ _, variable -> loggedNullableQuantityPrivateImpl(variable.name + "(SI value)", getValue,setValue) }


    
    public inline fun intList(
        crossinline getValue: () -> List<Int>,
        crossinline setValue: (List<Int>) -> Unit
    ): ReadWriteLoggableInput<List<Int>> =
        PropertyDelegateProvider{ _, variable -> loggedIntListPrivateImpl(variable.name, getValue,setValue) }

    public inline fun <D: AnyDimension> quantityList(
        crossinline getValue: () -> List<Quantity<D>>,
        crossinline setValue: (List<Quantity<D>>) -> Unit
    ): ReadWriteLoggableInput<List<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> loggedQuantityListPrivateImpl(variable.name + "(SI Value)", getValue,setValue) }

    public inline fun doubleList(
        crossinline getValue: () -> List<Double>,
        crossinline setValue: (List<Double>) -> Unit
    ): ReadWriteLoggableInput<List<Double>> =
        PropertyDelegateProvider{ _, variable -> loggedDoubleListPrivateImpl(variable.name, getValue, setValue) }

    public inline fun booleanList(
        crossinline getValue: () -> List<Boolean>,
        crossinline setValue: (List<Boolean>) -> Unit
    ): ReadWriteLoggableInput<List<Boolean>> =
        PropertyDelegateProvider{ _, variable -> loggedBooleanListPrivateImpl(variable.name, getValue, setValue) }

    public inline fun stringList(
        crossinline getValue: () -> List<String>,
        crossinline setValue: (List<String>) -> Unit
    ): ReadWriteLoggableInput<List<String>> =
        PropertyDelegateProvider{ _, variable -> loggedStringListPrivateImpl(variable.name, getValue, setValue) }


    public inline fun <T: AdvantageKitLoggable<T>> value(
        default: T,
        crossinline getValue: () -> T,
        crossinline setValue: (T) -> Unit
    ): ReadWriteLoggableInput<T> =
        PropertyDelegateProvider{ _, variable -> loggedGenericValuePrivateImpl(variable.name, default, getValue, setValue)}

    public inline fun <T: AdvantageKitLoggable<T>> nullableValue(
        default: T,
        crossinline getValue: () -> T?,
        crossinline setValue: (T?) -> Unit
    ): ReadWriteLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> loggedGenericNullableValuePrivateImpl(variable.name,default, getValue, setValue)}












    /**
     * Stores all log inputs of the [LoggableInputsProvider].
     */
    @PublishedApi
    internal val allLogInputs: MutableList<LoggableInputImpl> = mutableListOf()

    /**
     * Represents the internal implementation of a loggable input.
     */
    internal interface LoggableInputImpl {
        fun updateAndProcess()
    }

    init{
        ChargerRobot.runPeriodically{
            allLogInputs.forEach{
                it.updateAndProcess()
            }
        }
    }





    @PublishedApi
    internal inline fun loggedIntPrivateImpl(
        name: String,
        crossinline get: () -> Int,
        crossinline set: (Int) -> Unit = {},
    ): ReadWriteProperty<Any?,Int> = object: ReadWriteProperty<Any?,Int>, LoggableInputImpl {
        private var field = 0

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toLong())
            override fun fromLog(table: LogTable) { field = table.get(name,0).toInt()}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedDoublePrivateImpl(
        name: String,
        crossinline get: () -> Double,
        crossinline set: (Double) -> Unit = {}
    ): ReadWriteProperty<Any?,Double> = object: ReadWriteProperty<Any?,Double>, LoggableInputImpl{
        private var field = 0.0

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,0.0) }
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            set(value)
        }
    }

    @PublishedApi
    internal inline fun <D: AnyDimension> loggedQuantityPrivateImpl(
        name: String,
        crossinline get: () -> Quantity<D>,
        crossinline set: (Quantity<D>) -> Unit = {}
    ): ReadWriteProperty<Any?, Quantity<D>> = object: ReadWriteProperty<Any?, Quantity<D>>, LoggableInputImpl{
        private var field = Quantity<D>(0.0)

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.siValue)
            override fun fromLog(table: LogTable) { field = Quantity(table.get(name,0.0)) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedBooleanPrivateImpl(
        name: String,
        crossinline get: () -> Boolean,
        crossinline set: (Boolean) -> Unit = {}
    ): ReadWriteProperty<Any?,Boolean> = object: ReadWriteProperty<Any?,Boolean>, LoggableInputImpl{
        private var field = false

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,false) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedStringPrivateImpl(
        name: String,
        crossinline get: () -> String,
        crossinline set: (String) -> Unit = {}
    ): ReadWriteProperty<Any?,String> = object: ReadWriteProperty<Any?,String>, LoggableInputImpl{
        private var field = ""

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,"NOTHING") }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): String = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: String) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedNullableIntPrivateImpl(
        name: String,
        crossinline get: () -> Int?,
        crossinline set: (Int?) -> Unit = {}
    ): ReadWriteProperty<Any?, Int?> = object: ReadWriteProperty<Any?, Int?>, LoggableInputImpl{
        private var field: Int? = null

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name,(field ?: 0).toLong())
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get(name + "isValid", false)){
                    table.get(name,0).toInt()
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int? = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int?) {
            set(value)
        }
    }

    @PublishedApi
    internal inline fun loggedNullableDoublePrivateImpl(
        name: String,
        crossinline get: () -> Double?,
        crossinline set: (Double?) -> Unit = {}
    ): ReadWriteProperty<Any?, Double?> = object: ReadWriteProperty<Any?, Double?>, LoggableInputImpl{
        private var field: Double? = null

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name,field ?: 0.0)
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get(name + "isValid", false)){
                    table.get(name,0.0)
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Double? = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double?) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun <D: AnyDimension> loggedNullableQuantityPrivateImpl(
        name: String,
        crossinline get: () -> Quantity<D>?,
        crossinline set: (Quantity<D>?) -> Unit = {}
    ): ReadWriteProperty<Any?, Quantity<D>?> = object: ReadWriteProperty<Any?, Quantity<D>?>, LoggableInputImpl{
        private var field: Quantity<D>? = null

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name, field?.siValue ?: 0.0)
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get(name + "isValid",false)){
                    Quantity(table.get(name,0.0))
                }else{
                    null
                }
            }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D>? = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>?) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedIntListPrivateImpl(
        name: String,
        crossinline get: () -> List<Int>,
        crossinline set: (List<Int>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Int>> = object: ReadWriteProperty<Any?,List<Int>>, LoggableInputImpl{
        private var field = listOf<Int>()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.toLong()}.toLongArray())
            override fun fromLog(table: LogTable) { field = table.get(name,longArrayOf()).map{it.toInt()}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Int> = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Int>) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun <D: AnyDimension> loggedQuantityListPrivateImpl(
        name: String,
        crossinline get: () -> List<Quantity<D>>,
        crossinline set: (List<Quantity<D>>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Quantity<D>>> = object: ReadWriteProperty<Any?,List<Quantity<D>>>, LoggableInputImpl{
        private var field = listOf<Quantity<D>>()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.siValue}.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.get(name,doubleArrayOf()).map{Quantity(it)}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Quantity<D>> = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Quantity<D>>) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedDoubleListPrivateImpl(
        name: String,
        crossinline get: () -> List<Double>,
        crossinline set: (List<Double>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Double>> = object: ReadWriteProperty<Any?,List<Double>>, LoggableInputImpl{
        private var field = listOf<Double>()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.get(name, doubleArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Double> = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Double>) {
            set(value)
        }
    }

    @PublishedApi
    internal inline fun loggedBooleanListPrivateImpl(
        name: String,
        crossinline get: () -> List<Boolean>,
        crossinline set: (List<Boolean>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Boolean>> = object: ReadWriteProperty<Any?,List<Boolean>>, LoggableInputImpl{
        private var field = listOf<Boolean>()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toBooleanArray())
            override fun fromLog(table: LogTable) { field = table.get(name, booleanArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Boolean> = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Boolean>) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun loggedStringListPrivateImpl(
        name: String,
        crossinline get: () -> List<String>,
        crossinline set: (List<String>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<String>> = object: ReadWriteProperty<Any?,List<String>>, LoggableInputImpl{
        private var field = listOf<String>()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toTypedArray())
            override fun fromLog(table: LogTable) { field = table.get(name,arrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<String> = field

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<String>) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun <T: AdvantageKitLoggable<T>> loggedGenericValuePrivateImpl(
        name: String,
        default: T,
        crossinline get: () -> T,
        crossinline set: (T) -> Unit = {}
    ): ReadWriteProperty<Any?,T> = object: ReadWriteProperty<Any?,T>, LoggableInputImpl{
        private var field: T = default

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) { field.pushToLog(table,name) }

            override fun fromLog(table: LogTable) { field = field.getFromLog(table,name) }
        }

        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): T = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
            set(value)
        }
    }


    @PublishedApi
    internal inline fun <T: AdvantageKitLoggable<T>> loggedGenericNullableValuePrivateImpl(
        name: String,
        default: T,
        crossinline get: () -> T?,
        crossinline set: (T?) -> Unit = {}
    ): ReadWriteProperty<Any?,T?> = object: ReadWriteProperty<Any?,T?>, LoggableInputImpl{
        private var field: T? = default

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                val value = field
                if (value != null){
                    value.pushToLog(table,name)
                    table.put(name+"IsValid",true)
                }else{
                    default.pushToLog(table,name)
                    table.put(name+"IsValid",true)
                }
            }

            override fun fromLog(table: LogTable) {
                val value: T = field ?: default
                field = if (table.get(name+"IsValid",false)){
                    value.getFromLog(table,name)
                }else{
                    null
                }
            }
        }


        init{
            allLogInputs.add(this)
        }

        override fun updateAndProcess() {
            if (shouldUpdateInputs) field = get()
            Logger.processInputs(namespace, dummyInputs)
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): T? = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T?) {
            set(value)
        }
    }



}

