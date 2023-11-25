package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
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
 */
public class LinearTrapezoidProfile(
    public val constraints: LinearMotionConstraints,
    public val goalState: State,
    public val initialState: State = State(Distance(0.0),Velocity(0.0))
) {

    public companion object{
        public val None: LinearTrapezoidProfile = LinearTrapezoidProfile(
            LinearMotionConstraints(Velocity(0.0), Acceleration(0.0)),
            State(Distance(0.0), Velocity(0.0))
        )
    }

    /**
     * Represents a state of the Trapezoid Profile, with the respective [position] and [velocity].
     *
     * @see TrapezoidProfile.State
     */
    public data class State(
        val position: Distance,
        val velocity: Velocity
    ){
        public fun inUnit(distanceUnit: Distance, timeUnit: Time): TrapezoidProfile.State = TrapezoidProfile.State(
            position.inUnit(distanceUnit),
            velocity.inUnit(distanceUnit/timeUnit)
        )
    }

    public fun inUnit(distanceUnit: Distance, timeUnit: Time): TrapezoidProfile = TrapezoidProfile(
        constraints.inUnit(distanceUnit,timeUnit),
        goalState.inUnit(distanceUnit,timeUnit),
        initialState.inUnit(distanceUnit,timeUnit)
    )


    private val baseProfile = inUnit(meters,seconds)

    private var previousTime: Time = Time(0.0)

    public fun calculate(deltaT: Time): State =
        baseProfile.calculate(deltaT.inUnit(seconds)).ofUnit(meters,seconds)

    public fun calculateCurrentState(): State {
        val currentTime = fpgaTimestamp()
        return calculate(currentTime - previousTime).also{
            previousTime = currentTime
        }
    }

    public fun isFinished(t: Time): Boolean = baseProfile.isFinished(t.inUnit(seconds))

    public fun timeLeftUntil(target: Distance): Time = baseProfile.timeLeftUntil(target.inUnit(meters)).ofUnit(seconds)

    public fun totalTime(): Time = baseProfile.totalTime().ofUnit(seconds)


}



