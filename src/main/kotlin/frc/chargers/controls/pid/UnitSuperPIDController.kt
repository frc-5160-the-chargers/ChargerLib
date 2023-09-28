package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.controller.PIDController
import frc.chargers.commands.RunCommand
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.feedforward.Feedforward




/**
 * Wraps WPILib's [PIDController], adding various improvements, including unit support.
 * Essentially, this controller converts the input into its SI value before calculating PID,
 * then converts the output into SI Value form.
 *
 * @see SuperPIDController for an explanation of other added features.
 */
public class UnitSuperPIDController<I : AnyDimension, O : AnyDimension>(
    pidConstants: PIDConstants,
    private val getInput: () -> Quantity<I>,
    public val outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
    public val integralRange: ClosedRange<Quantity<O>> = outputRange,
    target: Quantity<I>,
    /**
     * Determines if the [UnitSuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    selfSustain: Boolean = false,
    /**
     * A lambda that returns the feedforward output of the controller it's in.
     * Intended to be callable as a function for other uses: I.E. controller.getFFOutput()
     */
    public val getFFOutput: UnitSuperPIDController<I,O>.() -> Quantity<O> = {Quantity(0.0)}
) : FeedbackController<Quantity<I>, Quantity<O>> {

    public companion object{
        /**
         * Fake Constructor of [UnitSuperPIDController]
         * which supports the native feedforward class with a separate input lambda,
         * instead of a Feedforward output lambda.
         *
         */
        public operator fun <I: AnyDimension, FI: AnyDimension, O: AnyDimension> invoke(
            pidConstants: PIDConstants,
            getInput: () -> Quantity<I>,
            outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
            integralRange: ClosedRange<Quantity<O>> = outputRange,
            target: Quantity<I>,
            selfSustain: Boolean = false,
            getFFInput: () -> Quantity<FI>,
            feedforward: Feedforward<Quantity<FI>,Quantity<O>> = Feedforward{Quantity(0.0)}
        ): UnitSuperPIDController<I,O> = UnitSuperPIDController(
            pidConstants,
            getInput,
            outputRange,
            integralRange,
            target,
            selfSustain
        ) { feedforward.calculate(getFFInput()) }

        /**
         * Fake Constructor of [UnitSuperPIDController]
         * where the input of the feedforward and the input of the PID controller are shared.
         *
         */
        public operator fun <I: AnyDimension, O: AnyDimension> invoke(
            pidConstants: PIDConstants,
            getInput: () -> Quantity<I>,
            outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
            integralRange: ClosedRange<Quantity<O>> = outputRange,
            target: Quantity<I>,
            selfSustain: Boolean = false,
            feedforward: Feedforward<Quantity<I>,Quantity<O>>
        ): UnitSuperPIDController<I,O> = invoke(
            pidConstants,
            getInput,
            outputRange,
            integralRange,
            target,
            selfSustain,
            getInput,
            feedforward,
        )
    }



    init{
        if(selfSustain){
            RunCommand{
                calculateOutput()
            }.schedule()
        }
    }


    private val pidController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setpoint = target.siValue
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public override fun calculateOutput(): Quantity<O> {
        val output = Quantity<O>(pidController.calculate(getInput().siValue)) + getFFOutput()
        return ensureInOutputRange(output)
    }


    private fun ensureInOutputRange(output: Quantity<O>): Quantity<O> {
        return output.coerceIn(outputRange)
    }

    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    override var target: Quantity<I>
        get() = Quantity(pidController.setpoint)
        set(target) {
            if (target.siValue != pidController.setpoint) {
                pidController.reset()
                pidController.setpoint = target.siValue
            }
        }

    /**
     * The PID Constants control exactly how the PID Controller attempts to reach the target.
     *
     * @see PIDConstants
     */
    public var constants: PIDConstants
        get() = pidController.constants
        set(pidConstants) {
            if (pidConstants != pidController.constants) {
                pidController.reset()
                pidController.constants = pidConstants
            }
        }

    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    override val error: Quantity<I>
        get() = Quantity(getInput().siValue - pidController.setpoint)
}