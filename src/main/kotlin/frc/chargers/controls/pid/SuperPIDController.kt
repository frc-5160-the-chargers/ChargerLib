package frc.chargers.controls.pid

import edu.wpi.first.math.controller.PIDController
import frc.chargers.commands.RunCommand
import frc.chargers.controls.Controller
import frc.chargers.controls.feedforward.Feedforward

/**
 * Wraps WPILib's [PIDController], adding various improvements.
 *
 * PID is a continuously-updating system that attempts to achieve a certain value ([.getTarget]/[.setTarget]) by varying another ([.calculateOutput]).
 * For example, you might use PID to a motor stay in a certain position, even when the motor is pushed, only by varying the power to the motor)
 *
 * See [here](https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html) for an explanation of PID.
 *
 * This class also adds feedforward([getFFOutput]), and adds the option to update the controller every loop ([selfSustain]).
 */
public class SuperPIDController(
    pidConstants: PIDConstants,
    private val getPIDInput: () -> Double,
    public val outputRange: ClosedRange<Double> = Double.NEGATIVE_INFINITY..Double.POSITIVE_INFINITY,
    public val integralRange: ClosedRange<Double> = outputRange,
    target: Double,
    /**
     * Determines if the [SuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    private val selfSustain: Boolean = false,
    /**
     * A lambda that returns the feedforward output of the controller it's in.
     * Intended to be callable as a function for other uses: I.E. controller.getFFOutput()
     */
    public val getFFOutput: SuperPIDController.() -> Double = {0.0}
): Controller<Double> {

    public companion object{
        /**
         * Fake Constructor of [SuperPIDController]
         * which supports the native feedforward class with an input lambda,
         * instead of a Feedforward output lambda.
         *
         */
        public operator fun invoke(
            pidConstants: PIDConstants,
            getInput: () -> Double,
            outputRange: ClosedRange<Double> = Double.NEGATIVE_INFINITY..Double.POSITIVE_INFINITY,
            integralRange: ClosedRange<Double> = outputRange,
            target: Double,
            selfSustain: Boolean = false,
            getFFInput: () -> Double,
            feedforward: Feedforward<Double,Double>
        ): SuperPIDController = SuperPIDController(
            pidConstants,
            getInput,
            outputRange,
            integralRange,
            target,
            selfSustain
        ) { feedforward.calculate(getFFInput()) }

        /**
         * Fake Constructor of [SuperPIDController]
         * where the input of the feedforward and the input of the PID controller are shared.
         *
         */
        public operator fun invoke(
            pidConstants: PIDConstants,
            getInput: () -> Double,
            outputRange: ClosedRange<Double> = Double.NEGATIVE_INFINITY..Double.POSITIVE_INFINITY,
            integralRange: ClosedRange<Double> = outputRange,
            target: Double,
            selfSustain: Boolean = false,
            feedforward: Feedforward<Double,Double>
        ): SuperPIDController = invoke(
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
            setpoint = target
            setIntegratorRange(integralRange.start, integralRange.endInclusive)
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public override fun calculateOutput(): Double {
        val output = pidController.calculate(getPIDInput()) + getFFOutput()
        return ensureInOutputRange(output)
    }


    private fun ensureInOutputRange(output: Double): Double {
        return output.coerceIn(outputRange)
    }

    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    public var target: Double
        get() = pidController.setpoint
        set(target) {
            if (target != pidController.setpoint) {
                pidController.reset()
                pidController.setpoint = target
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
    public val error: Double
        get() = getPIDInput() - pidController.setpoint
}