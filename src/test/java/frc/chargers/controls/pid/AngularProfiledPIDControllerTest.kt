package frc.chargers.controls.pid

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularAcceleration
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*
import kotlin.math.PI

internal class AngularProfiledPIDControllerTest {

    @Test
    fun `calculateOutput should be equivalent to WPILib PID controller`() {
        fun getInput() = PI / 2
        val target = 0.0
        val wpiController = ProfiledPIDController(3.0,0.0,0.0, TrapezoidProfile.Constraints(3.0,1.0))
        wpiController.enableContinuousInput(0.0.degrees.siValue,360.degrees.siValue)
        val chargerController = AngularProfiledPIDController(
            PIDConstants(3.0,0.0,0.0),
            { Angle(getInput()) },
            target = Angle(target),
            outputRange = (-12).volts..12.volts,
            continuousInputRange = Angle(0.0)..Angle(2 * PI),
            constraints = AngularMotionConstraints(AngularVelocity(3.0), AngularAcceleration(1.0)),
            feedforward = AngularMotorFF.None
        ) as FeedbackController<Angle,Voltage>

        val wpiOutput = wpiController.calculate(getInput(),target)
        val chargerOutput = chargerController.calculateOutput().siValue

        println(wpiOutput)
        println(chargerOutput)

        assertEquals(wpiOutput,chargerOutput)

    }

    @Test
    fun setTarget() {
    }
}