package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

public fun Double.revertIfInvalid(previousValue: Double): Double =
    if (isNaN() || isInfinite() || (abs(this) < 1.0e-4 && abs(previousValue) > 60.0) ) previousValue else this


public fun <D: AnyDimension> Quantity<D>.revertIfInvalid(previousValue: Quantity<D>): Quantity<D> =
    Quantity(siValue.revertIfInvalid(previousValue.siValue))