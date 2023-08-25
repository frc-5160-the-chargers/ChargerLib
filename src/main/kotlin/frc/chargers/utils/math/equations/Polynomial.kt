package frc.chargers.utils.math.equations

import kotlin.math.pow

/**
 * A class representing a polynomial function.
 *
 * Implements the (Double) -> (Double) interface, so can be called as a function.
 *
 * Coefficients are ordered from least to most significant;
 * for example, `Polynomial(1.0, 2.0, 3.0)` represents the polynomial
 * 1 + 2x + 3x^2.
 */
public data class Polynomial(val coefficients: List<Double>) : (Double) -> Double {
    public constructor(vararg coefficients: Double) : this(coefficients.toList())

    override fun invoke(x: Double): Double =
        coefficients
            .asSequence()
            .mapIndexed { power, coeff ->
                coeff * x.pow(power)
            }
            .sum()
}