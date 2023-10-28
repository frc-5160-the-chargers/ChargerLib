package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*


/**
 * Represents Gravity constants; used to supplement the [AngularMotorFF] and [LinearMotorFF].
 *
 * For example, Gravity.Arm represents the amount of voltage required
 * to keep an arm joint in a stable position.
 *
 * @see AngularMotorFF
 * @see LinearMotorFF
 */
public fun interface Gravity{
    public fun get(): Voltage







    public data object None: Gravity{
        override fun get(): Voltage = Voltage(0.0)

        override fun toString(): String = "NO gravity compensation"
    }

    public class Arm(public val kG: Voltage, private val getAngle: () -> Angle): Gravity{
        override fun get(): Voltage = kG * getAngle()

        override fun toString(): String = "Arm gravity compensation"
    }

    public class Elevator(public val kG: Voltage): Gravity{
        override fun get(): Voltage = kG

        override fun toString(): String = "Elevator Gravity Compensation"
    }
}
