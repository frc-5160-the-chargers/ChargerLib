package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.chargers.utils.math.equations.epsilonEquals
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class FeedforwardTest{
    val testcases = listOf(1.0,2.0,3.5,5.5, 10.5, 16.54234, 23.567, 1.1112, 4.0)
    val ks = 1.0
    val kg = 2.0
    val kv = 2.0
    val ka = 0.0

    @BeforeEach
    fun initialize(){
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `should be equal to wpilib simplemotorfeedforward`(){

        val chargerFF = Feedforward(
            AngularMotorFFConstants.fromSI(ks,kv,ka)
        )

        val wpiFF = SimpleMotorFeedforward(ks,kv,ka)

        for (testcase in testcases){
            assertEquals(
                chargerFF.calculate(AngularVelocity(testcase)).siValue epsilonEquals wpiFF.calculate(testcase),
                true
            )
        }
    }

    @Test
    fun `should be equal to wpilib armfeedforward`(){
        val angle = Angle(2.0)

        val chargerFF = Feedforward(
            ArmFFConstants.fromSI(ks,kg,kv,ka),
            {angle}
        )

        val wpiFF = ArmFeedforward(ks,kg,kv,ka)

        for (testcase in testcases){
            assertEquals(
                chargerFF.calculate(AngularVelocity(testcase)).siValue epsilonEquals wpiFF.calculate(angle.siValue,testcase),
                true
            )
        }
    }


}