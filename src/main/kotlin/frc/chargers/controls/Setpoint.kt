package frc.chargers.controls

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints

/**
 * Stores a produced Setpoint; with the appropriate setpoint value,
 * and additional produced output.
 */
public data class Setpoint<S: AnyDimension, O: AnyDimension>(
    /**
     * The value of the setpoint.
     */
    val value: Quantity<S>,
    /**
     * Additional control output provided by a [SetpointSupplier];
     * usually calculated via a feedforward.
     */
    val feedforwardOutput: Quantity<O>
)

/**
 * A class that can supply a certain setpoint to a [FeedbackController] or [Controller].
 */
public fun interface SetpointSupplier<S: AnyDimension, O: AnyDimension>{

    /**
     * Fetches a calculated [Setpoint],
     * with the appropriate target goal.
     */
    public fun getSetpoint(target: Quantity<S>): Setpoint<S, O>


    /**
     * A default setpoint supplier, with an optional feedforward
     * that is directly dependent on the target of the controller.
     *
     * For instance, in a velocity controller,
     * you would use this class and pass in the appropriate feedforward.
     */
    public class Default<I: AnyDimension, O: AnyDimension>(
        private val feedforward: Feedforward<I, O> = Feedforward{ Quantity(0.0) }
    ): SetpointSupplier<I,O>{
        override fun getSetpoint(target: Quantity<I>): Setpoint<I,O> =
            Setpoint(target,feedforward.calculate(target))
    }


    /**
     * Wraps WPILib's [TrapezoidProfile], acting as a [Setpoint] supplier
     * to a closed-loop controller,
     * with angular inputs.
     */
    public class AngularTrapezoidal(
        constraints: AngularMotionConstraints,
        private val feedforward: Feedforward<AngularVelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
    ): SetpointSupplier<AngleDimension, VoltageDimension>{

        public constructor(
            maxVelocity: AngularVelocity,
            maxAcceleration: AngularAcceleration,
            feedforward: Feedforward<AngularVelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
        ): this(AngularMotionConstraints(maxVelocity,maxAcceleration), feedforward)

        public constructor(
            constraints: TrapezoidProfile.Constraints,
            feedforward: Feedforward<AngularVelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
        ): this(AngularMotionConstraints(constraints), feedforward)


        private val trapProfile = TrapezoidProfile(constraints.siValue)
        private var currentState = TrapezoidProfile.State()
        private var previousT = fpgaTimestamp()

        /**
         * Fetches the appropriate [Setpoint], containing target position and FF output,
         * using a trapezoid profile, a target position and a target velocity of 0.
         */
        override fun getSetpoint(target: Angle): Setpoint<AngleDimension, VoltageDimension> =
            getSetpoint(target, AngularVelocity(0.0))

        /**
         * Fetches the appropriate [Setpoint], containing target position and FF output,
         * using a trapezoid profile, a target position and a target velocity.
         */
        public fun getSetpoint(
            targetPosition: Angle,
            targetVelocity: AngularVelocity
        ): Setpoint<AngleDimension, VoltageDimension> {
            val currentT = fpgaTimestamp()
            currentState = trapProfile.calculate(
                (currentT - previousT).inUnit(seconds),
                currentState,
                TrapezoidProfile.State(targetPosition.siValue,targetVelocity.siValue)
            )
            previousT = currentT
            return Setpoint(
                Angle(currentState.position),
                feedforward.calculate(AngularVelocity(currentState.velocity))
            )
        }

        /**
         * Fetches the current position that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointPosition: Angle
            get() = Angle(currentState.position)

        /**
         * Fetches the current velocity that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointVelocity: AngularVelocity
            get() = AngularVelocity(currentState.velocity)
    }

    public class LinearTrapezoidal(
        constraints: LinearMotionConstraints,
        private val feedforward: Feedforward<VelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
    ): SetpointSupplier<DistanceDimension, VoltageDimension>{

        public constructor(
            maxVelocity: Velocity,
            maxAcceleration: Acceleration,
            feedforward: Feedforward<VelocityDimension,VoltageDimension> = Feedforward{ Voltage(0.0) }
        ): this(LinearMotionConstraints(maxVelocity,maxAcceleration), feedforward)

        public constructor(
            constraints: TrapezoidProfile.Constraints,
            feedforward: Feedforward<VelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
        ): this(LinearMotionConstraints(constraints), feedforward)


        private val trapProfile = TrapezoidProfile(constraints.siValue)
        private var currentState = TrapezoidProfile.State()
        private var previousT = fpgaTimestamp()

        /**
         * Fetches the appropriate [Setpoint], containing target position and FF output,
         * using a trapezoid profile, a target position and a target velocity of 0.
         */
        override fun getSetpoint(target: Distance): Setpoint<DistanceDimension, VoltageDimension> =
            getSetpoint(target, Velocity(0.0))

        /**
         * Fetches the appropriate [Setpoint], containing target position and FF output,
         * using a trapezoid profile, a target position and a target velocity.
         */
        public fun getSetpoint(
            targetPosition: Distance,
            targetVelocity: Velocity
        ): Setpoint<DistanceDimension, VoltageDimension> {
            val currentT = fpgaTimestamp()
            currentState = trapProfile.calculate(
                (currentT - previousT).inUnit(seconds),
                currentState,
                TrapezoidProfile.State(targetPosition.siValue,targetVelocity.siValue)
            )
            previousT = currentT
            return Setpoint(
                Distance(currentState.position),
                feedforward.calculate(Velocity(currentState.velocity))
            )
        }

        /**
         * Fetches the current position that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointPosition: Distance
            get() = Distance(currentState.position)

        /**
         * Fetches the current velocity that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointVelocity: Velocity
            get() = Velocity(currentState.velocity)
    }


    public class AngularExponential(
        constraints: ExponentialProfile.Constraints,
        private val feedforward: Feedforward<AngularVelocityDimension, VoltageDimension>
    ): SetpointSupplier<AngleDimension, VoltageDimension>{
        private val expProfile = ExponentialProfile(constraints)
        private var currentState = ExponentialProfile.State(0.0,0.0)
        private var previousT = fpgaTimestamp()

        override fun getSetpoint(target: Angle): Setpoint<AngleDimension, VoltageDimension> =
            getSetpoint(target, AngularVelocity(0.0))

        /**
         * Fetches the appropriate [Setpoint], containing target position and FF output,
         * using an exponential profile, a target position and a target velocity.
         */
        public fun getSetpoint(
            targetPosition: Angle,
            targetVelocity: AngularVelocity
        ): Setpoint<AngleDimension, VoltageDimension>{
            val currentT = fpgaTimestamp()
            currentState = expProfile.calculate(
                (currentT - previousT).inUnit(seconds),
                currentState,
                ExponentialProfile.State(targetPosition.siValue,targetVelocity.siValue)
            )
            previousT = currentT
            return Setpoint(
                Angle(currentState.position),
                feedforward.calculate(AngularVelocity(currentState.velocity))
            )
        }

        /**
         * Fetches the current position that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointPosition: Angle
            get() = Angle(currentState.position)

        /**
         * Fetches the current velocity that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointVelocity: AngularVelocity
            get() = AngularVelocity(currentState.velocity)
    }

    public class LinearExponential(
        constraints: ExponentialProfile.Constraints,
        private val feedforward: Feedforward<VelocityDimension, VoltageDimension>
    ): SetpointSupplier<DistanceDimension, VoltageDimension> {
        private val expProfile = ExponentialProfile(constraints)
        private var currentState = ExponentialProfile.State(0.0,0.0)
        private var previousT = fpgaTimestamp()

        override fun getSetpoint(target: Distance): Setpoint<DistanceDimension, VoltageDimension> =
            getSetpoint(target, Velocity(0.0))

        /**
         * Fetches the appropriate [Setpoint], containing target position and FF output,
         * using an exponential profile, a target position and a target velocity.
         */
        public fun getSetpoint(
            targetPosition: Distance,
            targetVelocity: Velocity
        ): Setpoint<DistanceDimension, VoltageDimension>{
            val currentT = fpgaTimestamp()
            currentState = expProfile.calculate(
                (currentT - previousT).inUnit(seconds),
                currentState,
                ExponentialProfile.State(targetPosition.siValue,targetVelocity.siValue)
            )
            previousT = currentT
            return Setpoint(
                Distance(currentState.position),
                feedforward.calculate(Velocity(currentState.velocity))
            )
        }


        /**
         * Fetches the current position that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointPosition: Distance
            get() = Distance(currentState.position)

        /**
         * Fetches the current velocity that the [SetpointSupplier]
         * wants to direct the mechanism to.
         */
        public val setpointVelocity: Velocity
            get() = Velocity(currentState.velocity)
    }




}