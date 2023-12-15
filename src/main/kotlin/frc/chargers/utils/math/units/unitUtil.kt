package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.interop.transformWithSIValue
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.pow

/**
 * In kmeasure, Units are considered the same as quantities;
 * for example, the unit radians is actually a value, storing a Quantity(1.0).
 *
 * This "quantity" representation is meant to be considered a conversion factor,
 * to convert in and out of the unit itself.
 *
 * this typealias simply serves to increase code clarity within the library.
 */
public typealias KmeasureUnit<D> = Quantity<D>


/**
 * Gets an SI unit representation of a specific quantity and/or dimension.
 */
public fun <D: AnyDimension> siUnit(): KmeasureUnit<D> = KmeasureUnit(1.0)


public operator fun <D: AnyDimension> Quantity<D>.rem(other: Quantity<D>): Quantity<D> =
    Quantity(siValue%other.siValue)

public fun <D: AnyDimension> Quantity<D>.pow(exponent: Number): Quantity<D> =
    transformWithSIValue{it.pow(exponent.toDouble())}

public fun <D: AnyDimension> sqrt(value: Quantity<D>): Quantity<D> =
    Quantity(kotlin.math.sqrt(value.siValue))
