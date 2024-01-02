package frc.chargers.advantagekitextensions


import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.util.function.BooleanConsumer
import frc.chargers.framework.ChargerRobot
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
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
 * // you can selectively choose whether or not to update inputs;
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
 *      // and "Arm/nullablePropertyIsValid" logs whether or not the value is null or not.
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


    public fun <T: AdvantageKitLoggable<T>> value(getValue: () -> T): ReadOnlyLoggableInput<T> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericValue(variable.name,getValue)}
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
        getValue: () -> T,
        setValue: (T) -> Unit
    ): ReadWriteLoggableInput<T> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedGenericValue(variable.name,getValue,setValue)}
    public fun <T: AdvantageKitLoggable<T>> nullableValue(
        default: T,
        getValue: () -> T?,
        setValue: (T?) -> Unit
    ): ReadWriteLoggableInput<T?> =
        PropertyDelegateProvider{_, variable -> AutoLoggedGenericNullableValue(variable.name,default, getValue, setValue)}

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


    private inner class AutoLoggedInt(
        val name: String,
        val get: IntSupplier,
        val set: IntConsumer = IntConsumer{},
    ): ReadWriteProperty<Any?, Int>{
        private var field = get.asInt

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toLong())
            override fun fromLog(table: LogTable) { field = table.get(name,0) }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int = field

        init{
            ChargerRobot.runPeriodically{
                if (updateInputs) field = get.asInt
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int) {
            set.accept(value)
        }

    }


    private inner class AutoLoggedDouble(
        val name: String,
        val get: DoubleSupplier,
        val set: DoubleConsumer = DoubleConsumer{}
    ): ReadWriteProperty<Any?, Double>{
        private var field = get.asDouble

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,0.0) }
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = field

        init{
            ChargerRobot.runPeriodically{
                if (updateInputs) field = get.asDouble
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            set.accept(value)
        }
    }


    private inner class AutoLoggedQuantity<D: AnyDimension>(
        val name: String,
        val get: QuantitySupplier<D>,
        val set: QuantityConsumer<D> = QuantityConsumer{}
    ): ReadWriteProperty<Any?, Quantity<D>>{
        private var field = get.asQuantity()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.siValue)
            override fun fromLog(table: LogTable) { field = Quantity(table.get(name,0.0)) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = field

        init{
            ChargerRobot.runPeriodically{
                if (updateInputs) field = get.asQuantity()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>) {
            set.accept(value)
        }
    }

    private inner class AutoLoggedBoolean(
        val name: String,
        val get: BooleanSupplier,
        val set: BooleanConsumer = BooleanConsumer{}
    ): ReadWriteProperty<Any?,Boolean>{
        private var field = get.asBoolean
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,false) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = field

        init{
            ChargerRobot.runPeriodically{
                if (updateInputs) field = get.asBoolean
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
            set.accept(value)
        }

    }


    private inner class AutoLoggedString(
        val name: String,
        val get: () -> String,
        val set: (String) -> Unit = {}
    ): ReadWriteProperty<Any?,String>{
        private var field = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field)
            override fun fromLog(table: LogTable) { field = table.get(name,"NOTHING") }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): String = field

        init{
            ChargerRobot.runPeriodically{
                if (updateInputs) field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: String) {
            set(value)
        }
    }

    private inner class AutoLoggedNullableInt(
        val name: String,
        val get: () -> Int?,
        val set: (Int?) -> Unit = {}
    ): ReadWriteProperty<Any?, Int?>{
        private var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable){
                table.put(name,(field ?: 0).toLong())
                table.put(name + "isValid", field != null)
            }
            override fun fromLog(table: LogTable) {
                field = if (table.get(name + "isValid", false)){
                    table.get(name,0)
                }else{
                    null
                }
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Int? = field

        init{
            ChargerRobot.runPeriodically{
                if (updateInputs) field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int?) {
            set(value)
        }
    }



    private inner class AutoLoggedNullableDouble(
        val name: String,
        val get: () -> Double?,
        val set: (Double?) -> Unit = {}
    ): ReadWriteProperty<Any?, Double?>{
        private var field = get()

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
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double?) {
            set(value)
        }
    }



    private inner class AutoLoggedNullableQuantity<D: AnyDimension>(
        val name: String,
        val get: () -> Quantity<D>?,
        val set: (Quantity<D>?) -> Unit = {}
    ): ReadWriteProperty<Any?, Quantity<D>?>{
        private var field = get()
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
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: Quantity<D>?) {
            set(value)
        }
    }



    private inner class AutoLoggedIntList(
        val name: String,
        val get: () -> List<Int>,
        val set: (List<Int>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Int>>{
        private var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.toLong()}.toLongArray())
            override fun fromLog(table: LogTable) { field = table.get(name,longArrayOf()).map{it.toInt()}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Int> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Int>) {
            set(value)
        }
    }


    private inner class AutoLoggedQuantityList<D: AnyDimension>(
        val name: String,
        val get: () -> List<Quantity<D>>,
        val set: (List<Quantity<D>>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Quantity<D>>>{
        private var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.map{it.siValue}.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.get(name,doubleArrayOf()).map{Quantity(it)}}
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Quantity<D>> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Quantity<D>>) {
            set(value)
        }
    }


    private inner class AutoLoggedDoubleList(
        val name: String,
        val get: () -> List<Double>,
        val set: (List<Double>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Double>>{
        private var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toDoubleArray())
            override fun fromLog(table: LogTable) { field = table.get(name, doubleArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Double> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Double>) {
            set(value)
        }
    }


    private inner class AutoLoggedBooleanList(
        val name: String,
        val get: () -> List<Boolean>,
        val set: (List<Boolean>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<Boolean>>{
        private var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toBooleanArray())
            override fun fromLog(table: LogTable) { field = table.get(name, booleanArrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<Boolean> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<Boolean>) {
            set(value)
        }
    }

    private inner class AutoLoggedStringList(
        val name: String,
        val get: () -> List<String>,
        val set: (List<String>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<String>>{
        private var field = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,field.toTypedArray())
            override fun fromLog(table: LogTable) { field = table.get(name,arrayOf()).toList() }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<String> = field

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<String>) {
            set(value)
        }
    }

    private inner class AutoLoggedGenericValue<T: AdvantageKitLoggable<T>>(
        val name: String,
        val get: () -> T,
        val set: (T) -> Unit = {}
    ): ReadWriteProperty<Any?,T>{
        private var field: T = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) { field.pushToLog(table,name) }

            override fun fromLog(table: LogTable) { field = field.getFromLog(table,name) }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace,dummyInputs)
            }
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
    ): ReadWriteProperty<Any?,T?>{
        private var field: T? = get()


        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace,dummyInputs)
            }
        }

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


        override fun getValue(thisRef: Any?, property: KProperty<*>): T? = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T?) {
            set(value)
        }
    }


    private inner class AutoLoggedGenericValueList<T: AdvantageKitLoggable<T>>(
        val name: String,
        val default: T,
        val get: () -> List<T>,
        val set: (List<T>) -> Unit = {}
    ): ReadWriteProperty<Any?,List<T>>{
        private var field: List<T> = listOf(default)

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) {
                var counter = 1
                table.put("$name/totalItems", field.size.toLong())
                for (item in field){
                    item.pushToLog(table, "$name/Item#$counter")
                    counter++
                }
            }

            override fun fromLog(table: LogTable) {
                val totalItems = table.get("$name/totalItems", 0L)
                val newField = mutableListOf<T>()
                for (i in 1..totalItems){
                    newField.add(
                        default.getFromLog(table, "$name/Item#$i")
                    )
                }
                field = newField
            }
        }

        init{
            ChargerRobot.runPeriodically{
                field = get()
                Logger.processInputs(namespace, dummyInputs)
            }
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): List<T> = field

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: List<T>) {
            set(value)
        }
    }







}