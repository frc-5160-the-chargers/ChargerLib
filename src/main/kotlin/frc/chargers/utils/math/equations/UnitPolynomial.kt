package frc.chargers.utils.math.equations

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.pow

/**
 * A class representing a polynomial function, with Units support.
 *
 * Implements the (Quantity) -> (Quantity) interface, so can be called as a function.
 *
 * Coefficients are ordered from least to most significant;
 * for example, `Polynomial(1.0, 2.0, 3.0)` represents the polynomial
 * 1 + 2x + 3x^2.
 */
public data class UnitPolynomial<D: AnyDimension>(val coefficients: List<Double>) : (Quantity<D>) -> Quantity<D> {
    public constructor(vararg coefficients: Double) : this(coefficients.toList())

    override fun invoke(x: Quantity<D>): Quantity<D> =
        Quantity(
            coefficients
            .asSequence()
            .mapIndexed { power, coeff ->
                coeff * x.siValue.pow(power)
            }
            .sum()
        )
}