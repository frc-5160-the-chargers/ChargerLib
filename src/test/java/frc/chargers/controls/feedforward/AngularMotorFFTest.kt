package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*

class AngularMotorFFTest {

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `ensure calculate is equivalent to wpilib`() {
        val wpilibFF = SimpleMotorFeedforward(0.1,0.5)
        val chargerFF = AngularMotorFF(0.1.volts,0.5, angleUnit = radians)

        assertEquals(
            wpilibFF.calculate(5.0),
            chargerFF.calculate(5.0.radians / 1.seconds).inUnit(volts)
        )

    }
}