package frc.chargers.hardware.subsystemutils.swervedrive

import edu.wpi.first.hal.HAL
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class SwerveMotorsTest{
    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `configuration should be preserved`(){
        val motorHolder = sparkMaxSwerveMotors(
            neoSparkMax(0),
            neoSparkMax(1),
            neoSparkMax(2),
            neoSparkMax(3)
        ){inverted = true}

        assertEquals(motorHolder.topLeft.inverted, true)
    }
}