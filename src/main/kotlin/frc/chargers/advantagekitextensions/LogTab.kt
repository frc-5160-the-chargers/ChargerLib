package frc.chargers.advantagekitextensions

/*
import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import frc.chargers.framework.ChargerRobot
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty



public class LogTab(
    private val logGroup: String
){
    public fun entry(getValue: () -> Double): PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?,Double>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedDouble(variable.name, getValue) }

    public fun <D: AnyDimension> entry(getValue: () -> Quantity<D>): PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?,Quantity<D>>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedQuantity(variable.name + "(SI value)", getValue) }

    public fun entry(getValue: () -> Boolean): PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?,Boolean>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedBoolean(variable.name, getValue) }

    public fun entry(getValue: () -> String): PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?,String>> =
        PropertyDelegateProvider{ _, variable -> AutoLoggedString(variable.name, getValue) }




    private inner class AutoLoggedDouble(
        val name: String,
        val get: () -> Double
    ): ReadOnlyProperty<Any?, Double> {
        private var value = get()

        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,value)
            override fun fromLog(table: LogTable) { value = table.getDouble(name,value) }
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = value

        init{
            ChargerRobot.runPeriodically{
                value = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }
    }


    private inner class AutoLoggedQuantity<D: AnyDimension>(
        val name: String,
        val get: () -> Quantity<D>
    ): ReadOnlyProperty<Any?, Quantity<D>>{
        private var value = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,value.siValue)
            override fun fromLog(table: LogTable) { value = Quantity(table.getDouble(name,value.siValue)) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = value

        init{
            ChargerRobot.runPeriodically{
                value = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }
    }

    private inner class AutoLoggedBoolean(
        val name: String,
        val get: () -> Boolean
    ): ReadOnlyProperty<Any?,Boolean>{
        private var value = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,value)
            override fun fromLog(table: LogTable) { value = table.getBoolean(name,value) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = value

        init{
            ChargerRobot.runPeriodically{
                value = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

    }


    private inner class AutoLoggedString(
        val name: String,
        val get: () -> String
    ): ReadOnlyProperty<Any?,String>{
        private var value = get()
        private val dummyInputs = object: LoggableInputs{
            override fun toLog(table: LogTable) = table.put(name,value)
            override fun fromLog(table: LogTable) { value = table.getString(name,value) }
        }
        override fun getValue(thisRef: Any?, property: KProperty<*>): String = value

        init{
            ChargerRobot.runPeriodically{
                value = get()
                Logger.getInstance().processInputs(logGroup, dummyInputs)
            }
        }

    }
}

 */
