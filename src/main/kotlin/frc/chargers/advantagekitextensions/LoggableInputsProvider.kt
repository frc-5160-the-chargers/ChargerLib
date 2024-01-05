package frc.chargers.advantagekitextensions


import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.util.function.BooleanConsumer
import frc.chargers.framework.ChargerRobot
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.concurrent.locks.ReentrantLock
import java.util.function.*
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
 * A wrapper around AdvantageKit which manages logging and replaying loggable inputs
 * in a clean and descriptive way.
 *
 * Usage example:
 *
 * ```
 * // have 1 LoggableInputsProvider per subsystem/log category
 * val ArmLog = LoggableInputsProvider(namespace = "Arm")
 *
 * // you can selectively choose whether to update inputs;
 * // this is useful for subsystems that don't use sim and want replay functionality
 * // with 1 IO impl.
 * val IntakeLog = LoggableInputsProvider("Intake", updateInputs = RobotBase.isReal())
 *
 * public interface ArmIO{
 *      public fun lowLevelFunction(input: Angle){}
 *
 *      // instead of a separate "ArmIOInputs" class, low-level inputs are now part of
 *      // the io interface instead and are overriden by implementing classes
 *      public val current: Current
 *      public var appliedVoltage: Voltage
 *      public val otherProperty: Double
 *      public val nullableProperty: Double?
 * }
 *
 * public class ArmIOReal: ArmIO{
 *      // this value is now automatically logged under
 *      // "Arm/current(SI value)" and replayed from the same field.
 *      // no need to call processInputs or updateInputs periodically at all!
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
 *      // Int - LoggableInputsProvider.int{...}
 *      override val otherProperty by ArmLog.double{...}
 *
 *      // native support for kotlin nullables;
 *      // value logged under "Arm/nullableProperty",
 *      // and "Arm/nullablePropertyIsValid" logs whether the value is null or not.
 *      override val nullableProperty: Double? by ArmLog.nullableDouble{...}
 *      ...
 * }
 */
public class LoggableInputsProvider(
    public val namespace: String,
    public val updateInputs: Boolean = true
){
    public fun subgroup(group: String): LoggableInputsProvider =
        LoggableInputsProvider("$namespace/$group")

    /**
     * A [ReentrantLock] that automatically locks when the loggable inputs are updated.
     *
     */
    public val updateInputsLock: ReentrantLock = ReentrantLock()




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


    public fun <T: AdvantageKitLoggable<T>> value(default: T, getValue: () -> T): ReadOnlyLoggableInput<T> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericValue(variable.name,default, getValue)}
    public fun <T: AdvantageKitLoggable<T>> nullableValue(default: T, getValue: () -> T?): ReadOnlyLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericNullableValue(variable.name,default, getValue)}
    public fun <T: AdvantageKitLoggable<T>> valueList(
        default: T,
        getValue: () -> List<T>
    ): ReadOnlyLoggableInput<List<T>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedGenericValueList(variable.name, default, getValue) }





    public fun int(getValue: () -> Int, setValue: (Int) -> Unit): ReadWriteLoggableInput<Int> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedInt(variable.name, getValue,setValue) }
    public fun double(getValue: () -> Double, setValue: (Double) -> Unit): ReadWriteLoggableInput<Double> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDouble(variable.name, getValue,setValue) }
    public fun <D: AnyDimension> quantity(getValue: () -> Quantity<D>, setValue: (Quantity<D>) -> Unit): ReadWriteLoggableInput<Quantity<D>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantity(variable.name + "(SI value)", getValue,setValue) }
    public fun boolean(getValue: () -> Boolean, setValue: (Boolean) -> Unit): ReadWriteLoggableInput<Boolean> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBoolean(variable.name, getValue,setValue) }


    public fun string(getValue: () -> String, setValue: (String) -> Unit): ReadWriteLoggableInput<String> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedString(variable.name, getValue,setValue) }




    public fun nullableInt(getValue: () -> Int?, setValue: (Int?) -> Unit): ReadWriteLoggableInput<Int?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableInt(variable.name, getValue,setValue) }
    public fun nullableDouble(getValue: () -> Double?, setValue: (Double?) -> Unit): ReadWriteLoggableInput<Double?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableDouble(variable.name, getValue,setValue) }
    public fun <D: AnyDimension> nullableQuantity(getValue: () -> Quantity<D>?, setValue: (Quantity<D>?) -> Unit): ReadWriteLoggableInput<Quantity<D>?> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedNullableQuantity(variable.name + "(SI value)", getValue,setValue) }



    public fun intList(getValue: () -> List<Int>, setValue: (List<Int>) -> Unit): ReadWriteLoggableInput<List<Int>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedIntList(variable.name, getValue,setValue) }
    public fun <D: AnyDimension> quantityList(
        getValue: () -> List<Quantity<D>>,
        setValue: (List<Quantity<D>>) -> Unit
    ): ReadWriteLoggableInput<List<Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantityList(variable.name + "(SI Value)", getValue,setValue) }
    public fun doubleList(
        getValue: () -> List<Double>,
        setValue: (List<Double>) -> Unit
    ): ReadWriteLoggableInput<List<Double>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDoubleList(variable.name, getValue, setValue) }
    public fun booleanList(
        getValue: () -> List<Boolean>,
        setValue: (List<Boolean>) -> Unit
    ): ReadWriteLoggableInput<List<Boolean>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBooleanList(variable.name, getValue, setValue) }
    public fun stringList(
        getValue: () -> List<String>,
        setValue: (List<String>) -> Unit
    ): ReadWriteLoggableInput<List<String>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedStringList(variable.name, getValue, setValue) }


    public fun <T: AdvantageKitLoggable<T>> value(
        default: T,
        getValue: () -> T,
        setValue: (T) -> Unit
    ): ReadWriteLoggableInput<T> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedGenericValue(variable.name, default, getValue, setValue)}
    public fun <T: AdvantageKitLoggable<T>> nullableValue(
        default: T,
        getValue: () -> T?,
        setValue: (T?) -> Unit
    ): ReadWriteLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericNullableValue(variable.name, default, getValue, setValue)}

    public fun <T: AdvantageKitLoggable<T>> valueList(
        default: T,
        getValue: () -> List<T>,
        setValue: (List<T>) -> Unit
    ): ReadWriteLoggableInput<List<T>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedGenericValueList(variable.name, default, getValue, setValue) }



    /*
    These are no boxing overhead suppliers for Kmeasure Quantities.

    Note: For primitive type values, there is less overhead
    to using functional interfaces over function types; this is because when
    a type is passed in as a generic, it is automatically boxed, while functional interfaces
    are not boxed at all. Thus, for primitive types/value class types, we use
    functional interfaces to reduce the latency of this class.
     */

    private fun interface QuantitySupplier<D: AnyDimension>{
        fun asQuantity(): Quantity<D>
    }

    private fun interface QuantityConsumer<D: AnyDimension>{
        fun accept(value: Quantity<D>)
    }



    // Stores all auto logged items, in order to update them individually.
    private val allItems = mutableListOf<AutoLoggedItem>()

    /*
    Represents a Generic auto logged item.
     */
    private abstract inner class AutoLoggedItem {
        init{
            @Suppress("LeakingThis") // safe; works similarly to the java VirtualSubsystem
            allItems.add(this)
        }

        abstract val inputsProcessor: LoggableInputs

        abstract fun updateInputs()

    }

    init{
        ChargerRobot.runPeriodically{
            if (updateInputs){
                updateInputsLock.lock()
                allItems.forEach{ item ->
                    item.updateInputs()
                }
                updateInputsLock.unlock()
            }
            allItems.forEach{ item ->
                Logger.processInputs(namespace, item.inputsProcessor)
            }
        }
    }


    private inner class AutoLoggedInt(
        val name: String,
        val get: IntSupplier,
        val set: IntConsumer = IntConsumer{},
    ): ReadWriteProperty<Any?, Int>, AutoLoggedItem(){
        private var field = 0

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,0) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int = field

        override fun updateInputs() {
            field = get.asInt
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int) {
            set.accept(value)
        }

    }


    private inner class AutoLoggedDouble(
        val name: String,
        val get: DoubleSupplier,
        val set: DoubleConsumer = DoubleConsumer{}
    ): ReadWriteProperty<Any?, Double>, AutoLoggedItem(){
        private var field = 0.0

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,0.0) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = field

        override fun updateInputs() {
            field = get.asDouble
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            set.accept(value)
        }
    }


    private inner class AutoLoggedQuantity<D: AnyDimension>(
        val name: String,
        val get: QuantitySupplier<D>,
        val set: QuantityConsumer<D> = QuantityConsumer{}
    ): ReadWriteProperty<Any?, Quantity<D>>, AutoLoggedItem(){
        private var field = Quantity<D>(0.0)

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.siValue)
            override fun fromLog(table: LogTable) { field = Quantity(table.get(name,0.0)) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = field

        override fun updateInputs(){
            field = get.asQuantity()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>) {
            set.accept(value)
        }
    }

    private inner class AutoLoggedBoolean(
        val name: String,
        val get: BooleanSupplier,
        val set: BooleanConsumer = BooleanConsumer{}
    ): ReadWriteProperty<Any?,Boolean>, AutoLoggedItem(){
        private var field = false

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,false) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = field

        override fun updateInputs() {
            field = get.asBoolean
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
            set.accept(value)
        }
    }


    private inner class AutoLoggedString(
        val name: String,
        val get: () -> String,
        val set: (String) -> Unit = {}
    ): ReadWriteProperty<Any?,String>, AutoLoggedItem(){
        private var field = ""

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,"NOTHING") }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): String = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: String) {
            set(value)
        }
    }

    private inner class AutoLoggedNullableInt(
        val name: String,
        val get: () -> Int?,
        val set: (Int?) -> Unit = {}
    ): ReadWriteProperty<Any?, Int?>, AutoLoggedItem(){
        private var field: Int? = null

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable){
                // must store as value in order to pass null checks
                val currentValue = field
                if (currentValue == null){
                    table.put("$name/value", 0)
                    table.put("$name/isValid", false)
                }else{
                    table.put("$name/value", currentValue)
                    table.put("$name/isValid", true)
                }
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get("$name/isValid", false)){
                    table.get("$name/value",0)
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int? = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int?) {
            set(value)
        }
    }



    private inner class AutoLoggedNullableDouble(
        val name: String,
        val get: () -> Double?,
        val set: (Double?) -> Unit = {}
    ): ReadWriteProperty<Any?, Double?>, AutoLoggedItem(){
        private var field: Double? = null

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable){
                // must store as value in order to pass null checks
                val currentValue = field
                if (currentValue == null){
                    table.put("$name/value", 0.0)
                    table.put("$name/isValid", false)
                }else{
                    table.put("$name/value", currentValue)
                    table.put("$name/isValid", true)
                }
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get("$name/isValid", false)){
                    table.get("$name/value",0.0)
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Double? = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double?) {
            set(value)
        }
    }



    private inner class AutoLoggedNullableQuantity<D: AnyDimension>(
        val name: String,
        val get: () -> Quantity<D>?,
        val set: (Quantity<D>?) -> Unit = {}
    ): ReadWriteProperty<Any?, Quantity<D>?>, AutoLoggedItem(){
        private var field: Quantity<D>? = null

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable){
                // must store as value in order to pass null checks
                val currentValue = field
                if (currentValue == null){
                    table.put("$name/value", 0.0)
                    table.put("$name/isValid", false)
                }else{
                    table.put("$name/value", currentValue.siValue)
                    table.put("$name/isValid", true)
                }
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get("$name/isValid", false)){
                    Quantity(table.get("$name/value",0.0))
                }else{
                    null
                }
            }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D>? = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>?) {
            set(value)
        }
    }



    private inner class AutoLoggedIntList(
        val name: String,
        val get: () -> List<Int>,
        val set: (List<Int>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Int>>, AutoLoggedItem(){
        private var field = listOf<Int>()

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.toLong()}.toLongArray())
            override fun fromLog(table: LogTable) { field = table.get(name,longArrayOf()).map{it.toInt()}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Int> = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Int>) {
            set(value)
        }
    }


    private inner class AutoLoggedQuantityList<D: AnyDimension>(
        val name: String,
        val get: () -> List<Quantity<D>>,
        val set: (List<Quantity<D>>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Quantity<D>>>, AutoLoggedItem(){
        private var field = listOf<Quantity<D>>()

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.siValue}.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.get(name,doubleArrayOf()).map{Quantity(it)}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Quantity<D>> = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Quantity<D>>) {
            set(value)
        }
    }


    private inner class AutoLoggedDoubleList(
        val name: String,
        val get: () -> List<Double>,
        val set: (List<Double>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Double>>, AutoLoggedItem(){
        private var field = listOf<Double>()

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.get(name, doubleArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Double> = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Double>) {
            set(value)
        }
    }


    private inner class AutoLoggedBooleanList(
        val name: String,
        val get: () -> List<Boolean>,
        val set: (List<Boolean>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Boolean>>, AutoLoggedItem(){
        private var field = listOf<Boolean>()

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toBooleanArray())
            override fun fromLog(table: LogTable) { field = table.get(name, booleanArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Boolean> = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Boolean>) {
            set(value)
        }
    }

    private inner class AutoLoggedStringList(
        val name: String,
        val get: () -> List<String>,
        val set: (List<String>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<String>>, AutoLoggedItem(){
        private var field = listOf<String>()

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toTypedArray())
            override fun fromLog(table: LogTable) { field = table.get(name,arrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<String> = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<String>) {
            set(value)
        }
    }

    private inner class AutoLoggedGenericValue<T: AdvantageKitLoggable<T>>(
        val name: String,
        default: T,
        val get: () -> T,
        val set: (T) -> Unit = {}
    ): ReadWriteProperty<Any?,T>, AutoLoggedItem(){
        private var field: T = default

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) { field.pushToLog(table,name) }

            override fun fromLog(table: LogTable) { field = field.getFromLog(table,name) }
        }

        override fun updateInputs() {
            field = get()
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): T = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
            set(value)
        }
    }



    private inner class AutoLoggedGenericNullableValue<T: AdvantageKitLoggable<T>>(
        val name: String,
        val default: T,
        val get: () -> T?,
        val set: (T?) -> Unit = {}
    ): ReadWriteProperty<Any?,T?>, AutoLoggedItem(){
        private var field: T? = null

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable){
                (field ?: default).pushToLog(table, name)
                table.put("$name/isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get("$name/isValid", false)){
                    (field ?: default).getFromLog(table, name)
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): T? = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T?) {
            set(value)
        }
    }


    private inner class AutoLoggedGenericValueList<T: AdvantageKitLoggable<T>>(
        val name: String,
        val default: T,
        val get: () -> List<T>,
        val set: (List<T>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<T>>, AutoLoggedItem(){
        private var field: List<T> = listOf()

        override val inputsProcessor = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                var counter = 1
                table.put("$name/totalItems", field.size.toLong())
                for (item in field){
                    item.pushToLog(table, "$name/Item#$counter")
                    counter++
                }
            }

            override fun fromLog(table: LogTable) {
                val totalItems = table.get("$name/totalItems", 0)
                val newField = mutableListOf<T>()
                for (i in 1..totalItems){
                    newField.add(
                        default.getFromLog(table, "$name/Item#$i")
                    )
                }
                field = newField
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<T> = field

        override fun updateInputs() {
            field = get()
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<T>) {
            set(value)
        }
    }

}