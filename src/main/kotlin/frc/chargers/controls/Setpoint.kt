package frc.chargers.controls

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.framework.ChargerRobot
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints

/**
 * Stores a produced Setpoint; with the appropriate setpoint value,
 * and additional produced output.
 */
public data class Setpoint<I, O>(
    /**
     * The value of the setpoint.
     */
    val value: I,
    /**
     * Additional control output provided by a [SetpointSupplier];
     * usually calculated via a feedforward.
     */
    val feedforwardOutput: O
)

/**
 * A class that can supply a certain setpoint to a [FeedbackController] or [Controller].
 */
public fun interface SetpointSupplier<I, O>{

    /**
     * Fetches a calculated [Setpoint],
     * with the appropriate target goal.
     */
    public fun getSetpoint(target: I): Setpoint<I, O>


    /**
     * A default setpoint supplier, with an optional feedforward
     * that is directly dependent on the target of the controller.
     *
     * For instance, in a velocity controller,
     * you would use this class and pass in the appropriate feedforward.
     */
    public class Default<I: AnyDimension, O: AnyDimension>(
        private val feedforward: Feedforward<Quantity<I>,Quantity<O>> = Feedforward{ Quantity(0.0) }
    ): SetpointSupplier<Quantity<I>, Quantity<O>>{
        override fun getSetpoint(target: Quantity<I>): Setpoint<Quantity<I>, Quantity<O>> =
            Setpoint(Quantity(0.0),feedforward.calculate(target))
    }


    /**
     * Wraps WPILib's [TrapezoidProfile], acting as a [Setpoint] supplier
     * to a closed-loop controller,
     * with angular inputs.
     */
    public class AngularTrapezoidal(
        constraints: AngularMotionConstraints,
        private val feedforward: Feedforward<AngularVelocity,Voltage> = Feedforward{ Voltage(0.0) }
    ): SetpointSupplier<Angle, Voltage>{

        public constructor(
            maxVelocity: AngularVelocity,
            maxAcceleration: AngularAcceleration,
            feedforward: Feedforward<AngularVelocity,Voltage> = Feedforward{ Voltage(0.0) }
        ): this(AngularMotionConstraints(maxVelocity,maxAcceleration), feedforward)


        private val trapProfile = TrapezoidProfile(constraints.siValue)
        private var currentState = TrapezoidProfile.State()

        override fun getSetpoint(target: Angle): Setpoint<Angle, Voltage> =
            provideSetpoint(target, AngularVelocity(0.0))

        public fun provideSetpoint(position: Angle, velocity: AngularVelocity): Setpoint<Angle, Voltage> {
            currentState = trapProfile.calculate(
                ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                currentState,
                TrapezoidProfile.State(position.siValue,velocity.siValue)
            )
            return Setpoint(
                Angle(currentState.position),
                feedforward.calculate(AngularVelocity(currentState.velocity))
            )
        }
    }

    public class LinearTrapezoidal(
        constraints: LinearMotionConstraints,
        private val feedforward: Feedforward<Velocity,Voltage> = Feedforward{ Voltage(0.0) }
    ): SetpointSupplier<Distance, Voltage>{

        public constructor(
            maxVelocity: Velocity,
            maxAcceleration: Acceleration,
            feedforward: Feedforward<Velocity,Voltage> = Feedforward{ Voltage(0.0) }
        ): this(LinearMotionConstraints(maxVelocity,maxAcceleration), feedforward)


        private val trapProfile = TrapezoidProfile(constraints.siValue)
        private var currentState = TrapezoidProfile.State()

        override fun getSetpoint(target: Distance): Setpoint<Distance, Voltage> =
            provideSetpoint(target, Velocity(0.0))

        public fun provideSetpoint(position: Distance, velocity: Velocity): Setpoint<Distance, Voltage> {
            currentState = trapProfile.calculate(
                ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                currentState,
                TrapezoidProfile.State(position.siValue,velocity.siValue)
            )
            return Setpoint(
                Distance(currentState.position),
                feedforward.calculate(Velocity(currentState.velocity))
            )
        }
    }


    public class AngularExponential(
        constraints: ExponentialProfile.Constraints,
        private val feedforward: Feedforward<AngularVelocity,Voltage>
    ): SetpointSupplier<Angle,Voltage>{

        private val expProfile = ExponentialProfile(constraints)
        private var currentState = ExponentialProfile.State(0.0,0.0)

        override fun getSetpoint(target: Angle): Setpoint<Angle, Voltage> =
            provideSetpoint(target, AngularVelocity(0.0))
        public fun provideSetpoint(position: Angle, velocity: AngularVelocity): Setpoint<Angle, Voltage>{
            currentState = expProfile.calculate(
                ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                currentState,
                ExponentialProfile.State(position.siValue,velocity.siValue)
            )
            return Setpoint(
                Angle(currentState.position),
                feedforward.calculate(AngularVelocity(currentState.velocity))
            )
        }
    }

    public class LinearExponential(
        constraints: ExponentialProfile.Constraints,
        private val feedforward: Feedforward<Velocity,Voltage>
    ): SetpointSupplier<Distance,Voltage>{

        private val expProfile = ExponentialProfile(constraints)
        private var currentState = ExponentialProfile.State(0.0,0.0)

        override fun getSetpoint(target: Distance): Setpoint<Distance, Voltage> =
            provideSetpoint(target, Velocity(0.0))
        public fun provideSetpoint(position: Distance, velocity: Velocity): Setpoint<Distance, Voltage>{
            currentState = expProfile.calculate(
                ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                currentState,
                ExponentialProfile.State(position.siValue,velocity.siValue)
            )
            return Setpoint(
                Distance(currentState.position),
                feedforward.calculate(Velocity(currentState.velocity))
            )
        }
    }




}