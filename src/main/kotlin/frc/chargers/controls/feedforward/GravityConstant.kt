package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*



public sealed class Gravity private constructor(
    public val getValue: () -> Voltage = {Voltage(0.0)}
) {
    public object None: Gravity()

    public class Custom(getGravity: () -> Voltage): Gravity(getGravity)

    public class Arm(public val kG: Voltage, getAngle: () -> Angle): Gravity({kG * getAngle()})

    public class Elevator(public val kG: Voltage): Gravity({kG})
}