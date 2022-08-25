package frc.chargers.utils.pid

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.controller.PIDController

//TODO: Document properly
/**
 * Wraps WPILib's [PIDController], adding various improvements, including unit support.
 *
 * @see SuperPIDController for an explanation of other added features.
 */
public class UnitSuperPIDController<I : AnyDimension, O : AnyDimension>(
    pidConstants: PIDConstants,
    private val getInput: () -> Quantity<I>,
    public val outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
    public val integralRange: ClosedRange<Quantity<O>> = outputRange,
    target: Quantity<I>,
    public var feedForward: UnitFeedForward<I, O> = UnitFeedForward { _, _ -> Quantity(0.0) }
) {
    private val pidController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setpoint = target.siValue
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public fun calculateOutput(): Quantity<O> {
        val pidOutput = Quantity<O>(pidController.calculate(getInput().siValue))
        val fedForwardOutput = applyFeedforward(pidOutput)
        return ensureInOutputRange(fedForwardOutput)
    }

    private fun applyFeedforward(pidOutput: Quantity<O>): Quantity<O> {
        return pidOutput + feedForward.calculate(target, error)
    }

    private fun ensureInOutputRange(output: Quantity<O>): Quantity<O> {
        return output.coerceIn(outputRange)
    }

    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    public var target: Quantity<I>
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
    public val error: Quantity<I>
        get() = Quantity(getInput().siValue - pidController.setpoint)
}