package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.simulation.SimHooks
import frc.chargers.controls.SetpointSupplier
import frc.chargers.utils.math.equations.epsilonEquals
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


internal class SuperPIDControllerTest{

    @BeforeEach
    fun setup(){
        assert(HAL.initialize(500,0))
        SimHooks.getProgramStarted()
    }

    @Test
    fun `calculateOutput should be equivalent to WPILib PID controller`(){
        var input = 180.degrees
        val target = 0.0.degrees
        val kP = 3.0
        val kI = 0.0
        val kD = 0.1

        fun getInput() = input

        val wpiController = PIDController(kP,kI,kD)
        wpiController.enableContinuousInput(0.0.degrees.siValue,360.degrees.siValue)
        val chargerController = SuperPIDController<AngleDimension, VoltageDimension>(
            PIDConstants(kP,kI,kD),
            ::getInput,
            target,
            continuousInputRange = 0.degrees..360.degrees
        )

        for (i in 1..100){
            val wpiOutput = wpiController.calculate(getInput().siValue,target.siValue)
            val chargerOutput = chargerController.calculateOutput().siValue

            println(wpiOutput)
            println(chargerOutput)

            assertEquals(wpiOutput epsilonEquals chargerOutput, true)
            input -= 10.degrees
            SimHooks.stepTiming(0.02)
        }
    }

    @Test
    fun `calculateOutput should be equivalent to WPILib PID controller profiled`(){
        var input = 180.degrees
        val target = 0.0.degrees
        val kP = 3.0
        val kI = 0.0
        val kD = 0.1

        fun getInput() = input
        val constraints = AngularMotionConstraints(AngularVelocity(3.0), AngularAcceleration(1.0))

        val wpiController = ProfiledPIDController(kP,kI,kD, constraints.siValue)
        wpiController.enableContinuousInput(0.0.degrees.siValue,360.degrees.siValue)
        val chargerController = SuperPIDController<AngleDimension, VoltageDimension>(
            PIDConstants(kP,kI,kD),
            ::getInput,
            target,
            SetpointSupplier.AngularTrapezoidal(constraints),
            continuousInputRange = 0.degrees..360.degrees
        )

        for (i in 1..100){
            val wpiOutput = wpiController.calculate(getInput().siValue,target.siValue)
            val chargerOutput = chargerController.calculateOutput().siValue

            println(wpiOutput)
            println(chargerOutput)

            assertEquals(wpiOutput epsilonEquals chargerOutput, true)
            input -= 10.degrees
            SimHooks.stepTiming(0.02)
        }
    }
}