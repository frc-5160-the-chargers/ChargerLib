package frc.chargers.utils.math

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.interop.transformWithSIValue
import com.batterystaple.kmeasure.quantities.Quantity

public fun Double.preserveSign(initialValue: Double): Double =
    if ((this > 0 && initialValue < 0) || (this < 0 && initialValue > 0)) -this else this

public fun <D: AnyDimension> Quantity<D>.preserveSign(initialValue: Quantity<D>): Quantity<D> =
    this.transformWithSIValue{it.preserveSign(initialValue.siValue)}

