package frc.chargers.hardware.motorcontrol.rev

import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.BeforeEach

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class ChargerSparkMaxTest {

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `should factory default`(){
        val sparkMax = neoSparkMax(0){inverted = true}
        sparkMax.inverted = !sparkMax.inverted
        assertEquals(sparkMax.inverted, false)
    }
}