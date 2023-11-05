package frc.chargers.constants.tuning

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.units.KmeasureUnit
import frc.chargers.wpilibextensions.Alert
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

public typealias TunableDelegate<T> = PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?,T>>


/**
 * A class that interfaces with Smart Dashboard's tuning functionality.
 *
 * Acts as a wrapper around AdvantageKit's [LoggedDashboardNumber] and [LoggedDashboardBoolean],
 * which uses Kotlin's property delegates to support tunable
 * [Double]'s, [Quantity]'s, [Boolean]'s and [PIDConstants].
 *
 * Example usage:
 *
 * ```
 * class BasicSubsystem: SubsystemBase(){
 *      val tuner = DashboardTuner()
 *
 *
 *      val proximalJointConstants by tuner.pidConstants(
 *          default = PIDConstants(0.1,0.0,0.0),
 *          key = "proximalJointPIDConstants"
 *      )
 *
 *      val proximalJointOffset by tuner.quantity(
 *          default = 0.0.degrees,
 *          key = "proximalJointOffsetDegrees",
 *          logUnit = degrees
 *      )
 *
 * }
 * ```
 *
 */
public open class DashboardTuner{

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

        private const val DASH_KEY = "TunableValues"

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
                    println("Values have been refreshed for a Tunable Subsystem.")
                }
            }
        }
    }


    /**
     * A value that simply refreshes itself when a tunableValue is changed,
     * and tuning mode is enabled.
     */
    public fun <T: Any?> refreshWhenTuned(getValue: () -> T): ReadOnlyProperty<Any?, T> =
        object: ReadOnlyProperty<Any?, T> {

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
    public fun double(default: Double, key: String? = null): TunableDelegate<Double> = PropertyDelegateProvider{
        _, variable -> TunableDouble( key ?: variable.name, default)
    }

    private inner class TunableDouble(key: String, default: Double): ReadOnlyProperty<Any?, Double> {

        val dashNumber = LoggedDashboardNumber("$DASH_KEY/$key",default)
        private var value = default

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
    public fun <D: AnyDimension> quantity(default: Quantity<D>, key: String, logUnit: KmeasureUnit<D>): ReadOnlyProperty<Any?, Quantity<D>> =
        object : ReadOnlyProperty<Any?, Quantity<D>> {

            val dashNumber = LoggedDashboardNumber("$DASH_KEY/$key",default.inUnit(logUnit))
            private var value = default

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
    public fun boolean(default: Boolean, key: String): ReadOnlyProperty<Any?, Boolean> =
        object: ReadOnlyProperty<Any?, Boolean> {

            val dashBool = LoggedDashboardBoolean(key,default)
            private var value = default

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
    public fun pidConstants(default: PIDConstants, key: String): ReadOnlyProperty<Any?, PIDConstants> =
        object: ReadOnlyProperty<Any?, PIDConstants> {
            val kpDashNumber = LoggedDashboardNumber("$DASH_KEY/$key-kP",default.kP)
            val kiDashNumber = LoggedDashboardNumber("$DASH_KEY/$key-kI",default.kI)
            val kdDashNumber = LoggedDashboardNumber("$DASH_KEY/$key-kD",default.kD)

            private var value = default
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

    /**
     * Represents [PIDConstants] that can be tuned from the dashboard.
     */
    public fun pidConstants(kP: Double, kI: Double, kD: Double, key: String): ReadOnlyProperty<Any?,PIDConstants> =
        pidConstants(PIDConstants(kP,kI,kD),key)





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

}