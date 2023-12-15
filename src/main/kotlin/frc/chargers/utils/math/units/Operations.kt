package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.interop.transformWithSIValue
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.pow


public operator fun <D: AnyDimension> Quantity<D>.rem(other: Quantity<D>): Quantity<D> =
    Quantity(siValue%other.siValue)

public fun <D: AnyDimension> Quantity<D>.pow(exponent: Number): Quantity<D> =
    transformWithSIValue{it.pow(exponent.toDouble())}

public fun <D: AnyDimension> sqrt(value: Quantity<D>): Quantity<D> =
    Quantity(kotlin.math.sqrt(value.siValue))
