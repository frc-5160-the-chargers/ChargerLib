package frc.chargers.utils.math.equations

import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*
import kotlin.math.pow

internal class PolynomialTest {

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `polynomial output check`() {
        val input = 5.0
        val allCoefficients: List<List<Double>> = listOf(
            listOf(1.0,2.0,3.0),
            listOf(2.0,1.0,1000.0),
            listOf(5000.0,0.0,5160.66)
        )
        for (coefficients in allCoefficients){
            var computedOutput = 0.0
            computedOutput += coefficients[0] * input.pow(2)
            computedOutput += coefficients[1] * input
            computedOutput += coefficients[2]

            assertEquals(
                Polynomial(coefficients)(x = input),
                computedOutput
            )
        }
    }
}