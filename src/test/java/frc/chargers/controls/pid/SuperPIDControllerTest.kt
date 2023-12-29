package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.SetpointSupplier
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import kotlin.math.PI


internal class SuperPIDControllerTest{

    @BeforeEach
    fun setup(){
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `calculateOutput should be equivalent to WPILib PID controller`(){
        val getInput = {180.degrees}
        val target = 0.degrees
        val wpilibController = PIDController(0.2,0.0,0.0)
        wpilibController.enableContinuousInput(0.0.degrees.siValue, 360.degrees.siValue)


        val chargerPIDcontroller = SuperPIDController<AngleDimension,ScalarDimension>(
            PIDConstants(0.2,0.0,0.0),
            getInput,
            target = target,
            continuousInputRange = 0.degrees..360.degrees
        )

        //wpilibController.setpoint = target.siValue
        assertEquals(wpilibController.calculate(getInput().siValue, target.siValue), chargerPIDcontroller.calculateOutput().siValue)
    }

    @Test
    fun `calculateOutput should be equivalent to WPILib PID controller profiled`(){
        fun getInput() = PI / 2
        val target = 0.0
        val constraints = AngularMotionConstraints(AngularVelocity(3.0), AngularAcceleration(1.0))
        val kP = 3.0
        val kI = 0.0
        val kD = 0.0


        val wpiController = ProfiledPIDController(kP,kI,kD, TrapezoidProfile.Constraints(3.0,1.0))
        wpiController.enableContinuousInput(0.0.degrees.siValue,360.degrees.siValue)
        val chargerController = SuperPIDController<AngleDimension, VoltageDimension>(
            PIDConstants(kP,kI,kD),
            {getInput().ofUnit(radians)},
            Angle(target),
            SetpointSupplier.AngularTrapezoidal(constraints),
            continuousInputRange = 0.degrees..360.degrees
        )

        val wpiOutput = wpiController.calculate(getInput(),target)
        val chargerOutput = chargerController.calculateOutput().siValue

        println(wpiOutput)
        println(chargerOutput)

        assertEquals(wpiOutput,chargerOutput)

    }
}