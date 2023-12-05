package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit

/*
NOTE TO SELF:
WPILib API is changing in 2024, so class will have to be changed accordingly:

fyi, the API is changing for 2024 so that calculate() takes initial position, unprofiled goal, and dt, and returns profiled setpoint for dt in the future. That way, you can reuse the instance.
https://github.com/wpilibsuite/allwpilib/pull/5457
 */

/**
 * A wrapper for WPILib's [TrapezoidProfile] class, adding support for angular units.
 * For instance, the constraints are measured with max angular acceleration and max angular velocity.
 *
 */
public class AngularTrapezoidProfile(
    public val constraints: AngularMotionConstraints
) {

    public companion object{
        public val None: AngularTrapezoidProfile = AngularTrapezoidProfile(
            AngularMotionConstraints(AngularVelocity(0.0),AngularAcceleration(0.0))
        )
    }

    /**
     * Represents a state of the Trapezoid Profile, with the respective [position] and [velocity].
     *
     * @see TrapezoidProfile.State
     */
    public data class State(
        val position: Angle,
        val velocity: AngularVelocity
    ){
        public fun inUnit(angleUnit: Angle, timeUnit: Time): TrapezoidProfile.State = TrapezoidProfile.State(
            position.inUnit(angleUnit),
            velocity.inUnit(angleUnit/timeUnit)
        )
    }

    public fun inUnit(angleUnit: Angle, timeUnit: Time): TrapezoidProfile = TrapezoidProfile(
        constraints.inUnit(angleUnit,timeUnit)
    )


    private val baseProfile = inUnit(radians,seconds)

    private var previousTime: Time = Time(0.0)

    public fun calculate(deltaT: Time, goalState: State, currentState: State): State =
        baseProfile.calculate(deltaT.inUnit(seconds), goalState.inUnit(radians,seconds), currentState.inUnit(radians,seconds)).ofUnit(radians,seconds)

    public fun calculateCurrentState(currentState: State, goalState: State): State{
        val currentTime = fpgaTimestamp()
        return calculate(currentTime - previousTime, currentState, goalState).also{
            previousTime = currentTime
        }
    }

    public fun isFinished(t: Time): Boolean = baseProfile.isFinished(t.inUnit(seconds))

    public fun timeLeftUntil(target: Angle): Time = baseProfile.timeLeftUntil(target.inUnit(radians)).ofUnit(seconds)

    public fun totalTime(): Time = baseProfile.totalTime().ofUnit(seconds)


}


