package frc.chargers.hardware.subsystems

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.wpilibextensions.Alert
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty


/**
 * Represents a Subsystem with tunable values.
 */
public abstract class TunableSubsystem: SubsystemBase(){

    public companion object{
        
        public var tuningMode: Boolean = false
            set(value){
                if (!value){
                    field = false
                    isCompAlert.active = false
                }else if (DriverStation.isFMSAttached()){
                    isCompAlert.active = true
                }else{
                    field = true
                    tuningModeEnabledAlert.active = true
                    isCompAlert.active = false
                }
            }

        internal const val DASH_KEY = "TunableValues"

        private val isCompAlert = Alert.warning(text = "Tuning mode WAS NOT SET: It looks like you're in a match right now.")
        private val tuningModeEnabledAlert = Alert.warning(text = "Tuning mode is enabled; Expect loop times to be greater. ")
    }

    init{
        ChargerRobot.addToPeriodicLoop{
            if (tuningMode){
                val updateStatus: List<Boolean> = allTunables.map{
                    it.needsUpdate()
                }

                if (true in updateStatus){
                    allTunables.forEach{
                        it.updateValue()
                    }
                    allRefreshables.forEach{
                        it.refresh()
                    }
                }
            }
        }
    }


    /**
     * represents a generic value that can be refreshed.
     */
    private class Tunable(
        val updateValue: () -> Unit,
        val needsUpdate: () -> Boolean
    )

    /**
     * Represents a generic item which depends on tunables, and can be refreshed when nessecary.
     */
    private fun interface Refreshable{
        fun refresh()
    }

    private val allTunables: MutableList<Tunable> = mutableListOf()
    private val allRefreshables: MutableList<Refreshable> = mutableListOf()


    /**
     * A value that simply refreshes itself when a tunableValue is changed,
     * and tuning mode is enabled.
     */
    protected fun <T: Any?> refreshable(getValue: () -> T): ReadOnlyProperty<Any?,T> =
        object: ReadOnlyProperty<Any?,T>{

            private var value: T = getValue()

            init{
                allRefreshables.add(
                    Refreshable{ value = getValue() }
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): T = value
        }

    /**
     * A property delegate that represents a tunable [Double].
     *
     * @see LoggedDashboardNumber
     */
    protected fun tunableDouble(defaultValue: Double, key: String): ReadOnlyProperty<Any?,Double> =
        object: ReadOnlyProperty<Any?,Double>{

            val dashNumber = LoggedDashboardNumber("$DASH_KEY/$key",defaultValue)
            private var value = defaultValue

            init{
                allTunables.add(
                    Tunable(
                        {value = dashNumber.get()},
                        {dashNumber.get() == value}
                    )
                )
            }


            override fun getValue(thisRef: Any?, property: KProperty<*>): Double = value
        }

    /**
     * A property delegate that represents a tunable [Quantity].
     *
     * @see Quantity
     * @see LoggedDashboardNumber
     */
    protected fun <D: AnyDimension> tunableQuantity(defaultValue: Quantity<D>, key: String, logUnit: Quantity<D>): ReadOnlyProperty<Any?,Quantity<D>> =
        object : ReadOnlyProperty<Any?,Quantity<D>>{

            val dashNumber = LoggedDashboardNumber("$DASH_KEY/$key",defaultValue.inUnit(logUnit))
            private var value = defaultValue

            init{
                allTunables.add(
                    Tunable(
                        {value = dashNumber.get().ofUnit(logUnit)},
                        {dashNumber.get() == value.inUnit(logUnit)}
                    )
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = value

        }

    /**
     * A property delegate that represents a tunable [Boolean].
     *
     * @see LoggedDashboardBoolean
     */
    protected fun tunableBoolean(defaultValue: Boolean, key: String): ReadOnlyProperty<Any?,Boolean> =
        object: ReadOnlyProperty<Any?,Boolean>{

            val dashBool = LoggedDashboardBoolean(key,defaultValue)
            private var value = defaultValue

            init{
                allTunables.add(
                    Tunable(
                        {value = dashBool.get()},
                        {value == dashBool.get()}
                    )
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = value
        }

    /**
     * Represents [PIDConstants] that can be tuned from the dashboard.
     */
    protected fun tunablePIDConstants(defaultValue: PIDConstants, key: String): ReadOnlyProperty<Any?,PIDConstants> =
        object: ReadOnlyProperty<Any?,PIDConstants>{
            val kpDashNumber = LoggedDashboardNumber("$DASH_KEY/$key/kP",defaultValue.kP)
            val kiDashNumber = LoggedDashboardNumber("$DASH_KEY/$key/kI",defaultValue.kI)
            val kdDashNumber = LoggedDashboardNumber("$DASH_KEY/$key/kD",defaultValue.kD)

            private var value = defaultValue
            // ap test stuff; cs and math
            private fun getConstants() = PIDConstants(
                kpDashNumber.get(),
                kiDashNumber.get(),
                kdDashNumber.get()
            )

            init{
                allTunables.add(
                    Tunable(
                        { value = getConstants() },
                        { value != getConstants() }
                    )
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): PIDConstants = value

        }

}







