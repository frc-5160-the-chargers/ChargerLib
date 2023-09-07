package frc.chargers.utils.math.equations

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import kotlin.math.pow

/**
 * A class representing a polynomial function, with Units support.
 *
 * Example usage:
 *
 * val pol = UnitPolynomial([degrees] to [volts], listOf(5.0,6.1,7.2))
 *
 * val pol = UnitPolynomial([degrees] to [volts], 5.1,6.1,7.2)
 *
 * Coefficients are ordered from least to most significant.
 *
 * @see Polynomial
 */
public data class UnitPolynomial<I: AnyDimension, O: AnyDimension>(
    val unitsUsed: Pair<Quantity<I>, Quantity<O>> = Quantity<I>(1.0) to Quantity<O>(1.0),
    val coefficients: List<Double>
) : (Quantity<I>) -> Quantity<O> {
    public constructor(
        unitsUsed: Pair<Quantity<I>, Quantity<O>> = Quantity<I>(1.0) to Quantity<O>(1.0),
        vararg coefficients: Double
    ) : this(unitsUsed, coefficients.toList())

    override fun invoke(x: Quantity<I>): Quantity<O> =
        coefficients
        .asSequence()
        .mapIndexed { power, coeff ->
            coeff * x.inUnit(unitsUsed.first).pow(power)
        }
        .sum()
        .ofUnit(unitsUsed.second)
}

