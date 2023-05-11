package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

public fun interface UnitOutputExtension<I : AnyDimension, O : AnyDimension> {
    public fun calculate(target: Quantity<I>, error: Quantity<I>): Quantity<O>
}