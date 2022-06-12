package frc.robot.utils.pid

import com.batterystaple.kmeasure.Dimension
import com.batterystaple.kmeasure.DimensionedQuantity
import com.batterystaple.kmeasure.ScalarDimension
import com.batterystaple.kmeasure.value
import edu.wpi.first.math.controller.PIDController

//TODO: Document properly
/**
 * Wraps WPILib's [PIDController], adding various improvements, including unit support.
 *
 * @see SuperPIDController for an explanation of other added features.
 */
public class UnitSuperPIDController<I : Dimension, O : Dimension>(
    pidConstants: PIDConstants,
    private val getInput: () -> DimensionedQuantity<I>,
    public val outputRange: ClosedRange<DimensionedQuantity<O>> = DimensionedQuantity<O>(Double.NEGATIVE_INFINITY)..DimensionedQuantity<O>(Double.POSITIVE_INFINITY),
    target: DimensionedQuantity<I>,
    public var feedForward: UnitFeedForward<I, O> = UnitFeedForward { _, _ -> DimensionedQuantity(0.0) }
) {
    private val pidController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setpoint = target.siValue
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public fun calculateOutput(): DimensionedQuantity<O> {
        val pidOutput = DimensionedQuantity<O>(pidController.calculate(getInput().siValue))
        val fedForwardOutput = applyFeedforward(pidOutput)
        return ensureInOutputRange(fedForwardOutput)
    }

    private fun applyFeedforward(pidOutput: DimensionedQuantity<O>): DimensionedQuantity<O> {
        return pidOutput + feedForward.calculate(target, error)
    }

    private fun ensureInOutputRange(output: DimensionedQuantity<O>): DimensionedQuantity<O> {
        return output.coerceIn(outputRange)
    }

    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    public var target: DimensionedQuantity<I>
        get() = DimensionedQuantity(pidController.setpoint)
        set(target) {
            pidController.reset()
            pidController.setpoint = target.siValue
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
    public val error: DimensionedQuantity<I>
        get() = DimensionedQuantity(getInput().siValue - pidController.setpoint)
}