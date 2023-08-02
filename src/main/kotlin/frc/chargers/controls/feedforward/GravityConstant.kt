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
public sealed class Gravity private constructor(
    public val getOutput: () -> Voltage = {Voltage(0.0)}
) {
    public object None: Gravity()

    public class Custom(getGravity: () -> Voltage): Gravity(getGravity)

    public class Arm(public val kG: Voltage, getAngle: () -> Angle): Gravity({kG * getAngle()})

    public class Elevator(public val kG: Voltage): Gravity({kG})
}