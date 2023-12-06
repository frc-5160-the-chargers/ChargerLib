package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Velocity
import com.pathplanner.lib.PathConstraints
import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*

class LinearMotionConstraintsTest {

    @BeforeEach
    fun setUp() {
        assert(
            HAL.initialize(500,0)
        )
    }

    @Test
    fun toPathConstraints() {
        val motionConstraints = LinearMotionConstraints(Velocity(2.0),Acceleration(2.0))
        val motionConstraintsSIValue = motionConstraints.toPathConstraints()
        val pathConstraints = PathConstraints(2.0,2.0)
        assertEquals(motionConstraintsSIValue.maxVelocity,pathConstraints.maxVelocity)
        assertEquals(motionConstraintsSIValue.maxAcceleration,pathConstraints.maxAcceleration)
    }
}