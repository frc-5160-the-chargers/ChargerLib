package frc.chargers.utils.math

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.hal.HAL
import frc.chargers.utils.a
import frc.chargers.utils.math.equations.epsilonEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*

internal class InputModulusKtTest {

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    private fun basicStandardize(input: Double): Double = if (input < 0){
        (input % 360.0) + 360.0
    }else{
        input % 360.0
    }

    @Test
    fun inputModulus() {
        val inputs = a[90.0,70.0,20.0,30.0,180.0,270.0,-90.0, -45.0, -30.0]

        for (testcase in inputs){
            assertEquals(
                basicStandardize(testcase) epsilonEquals testcase.inputModulus(0.0..360.0),
                true
            )
        }
    }

    @Test
    fun inputModulusWithUnits() {
        val inputs = a[90.0,70.0,20.0,30.0,180.0,270.0, -50.0, -60.0, -120.0, -190.0]
        for (testcase in inputs){
            val testingInput = testcase.ofUnit(degrees).inputModulus(0.degrees..360.degrees).inUnit(degrees)
            val actualInput = basicStandardize(testcase)

            assertEquals(
                testingInput epsilonEquals actualInput,
                true
            )
        }
    }
}