package frc.robot.utils.pid

import edu.wpi.first.math.controller.PIDController

/**
 * Wraps WPILib's [PIDController], adding various improvements.
 *
 * PID is a continuously-updating system that attempts to achieve a certain value ([.getTarget]/[.setTarget]) by varying another ([.calculateOutput]).
 * For example, you might use PID to a motor stay in a certain position, even when the motor is pushed, only by varying the power to the motor)
 *
 * See [here](https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html) for an explanation of PID.
 *
 * This class also adds feedforward capability, an augmentation to PID that can increase how quickly it reaches the target value.
 *
 * See [here](https://www.controleng.com/articles/feed-forwards-augment-pid-control/) for an explanation of feedforward.
 */
public class SuperPIDController(
    pidConstants: PIDConstants,
    private val getInput: () -> Double,
    public val outputRange: ClosedRange<Double> = Double.NEGATIVE_INFINITY..Double.POSITIVE_INFINITY,
    target: Double,
    public var feedForward: FeedForward = FeedForward { _, _ -> 0.0 }
) {
    private val pidController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setpoint = target
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public fun calculateOutput(): Double {
        val pidOutput = pidController.calculate(getInput())
        val fedForwardOutput = applyFeedforward(pidOutput)
        return ensureInOutputRange(fedForwardOutput)
    }

    private fun applyFeedforward(pidOutput: Double): Double {
        return pidOutput + feedForward.calculate(target, error)
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
            pidController.reset()
            pidController.setpoint = target
        }

    /**
     * The PID Constants control exactly how the PID Controller attempts to reach the target.
     *
     * @see PIDConstants
     */
    public var constants: PIDConstants
        get() = pidController.constants
        set(pidConstants) {
            pidController.reset()
            pidController.constants = pidConstants
        }

    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    public val error: Double
        get() = getInput() - pidController.setpoint
}