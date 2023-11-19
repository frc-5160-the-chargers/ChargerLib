package frc.chargers.utils.math

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.hal.HAL
import frc.chargers.utils.a
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*
import kotlin.math.abs

class InputModulusKtTest {

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    fun basicStandardize(input: Double): Double = if (input < 0){
        (input % 360.0) + 0.0
    }else{
        input % 360.0
    }

    @Test
    fun inputModulus() {
        val inputs = a[90.0,70.0,20.0,30.0,180.0,270.0]

        for (testcase in inputs){
            assertEquals(
                basicStandardize(testcase),
                testcase.inputModulus(0.0..360.0)
            )
        }
    }

    @Test
    fun inputModulusWithUnits() {
        val inputs = a[90.0,70.0,20.0,30.0,180.0,270.0]
        for (testcase in inputs){
            val testingInput = testcase.degrees.inputModulus(0.degrees..360.degrees).inUnit(degrees)
            val actualInput = basicStandardize(testcase)

            assertEquals(
                abs(testingInput - actualInput) < 0.0001,
                true
            )
        }
    }
}