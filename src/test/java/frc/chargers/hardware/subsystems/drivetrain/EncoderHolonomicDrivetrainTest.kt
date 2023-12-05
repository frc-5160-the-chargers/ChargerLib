package frc.chargers.hardware.subsystems.drivetrain

import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach

class EncoderHolonomicDrivetrainTest {

    @BeforeEach
    fun setUp(){
        assert(
            HAL.initialize(500,0)
        )
    }

    @Test
    fun moduleStatesTest(){

    }

    @Test
    fun swerveDrive() {
    }
}