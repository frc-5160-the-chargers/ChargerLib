package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.controller.PIDController
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class UnitSuperPIDControllerTest{

    @BeforeEach
    fun setup(){
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `calculate output vs wpilib pid controller`(){
        val getInput = {180.degrees}
        val target = 0.degrees
        val wpilibController = PIDController(0.2,0.0,0.0)
        wpilibController.enableContinuousInput(0.0.degrees.siValue, 360.degrees.siValue)


        val chargerPIDcontroller = UnitSuperPIDController<AngleDimension,ScalarDimension>(
            PIDConstants(0.2,0.0,0.0),
            getInput,
            target = 0.degrees,
            continuousInputRange = 0.0.degrees..360.degrees
        )

        //wpilibController.setpoint = target.siValue
        assertEquals(wpilibController.calculate(getInput().siValue, target.siValue), chargerPIDcontroller.calculateOutput().siValue)
    }
}