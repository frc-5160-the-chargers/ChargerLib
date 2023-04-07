package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.Controller

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
) : Controller<Quantity<O>> {
    // note: sets the base PID Controller to public to allow stuff such as setContinuousInput, etc. etc.
    // - Daniel
    public val basePID: PIDController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setpoint = target.siValue
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
        }

    public fun enableContinuousInput(bottomValue: Double, topValue: Double){
        basePID.enableContinuousInput(bottomValue,topValue)
    }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public override fun calculateOutput(): Quantity<O> {
        val pidOutput = Quantity<O>(basePID.calculate(getInput().siValue))
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
        get() = Quantity(basePID.setpoint)
        set(target) {
            if (target.siValue != basePID.setpoint) {
                basePID.reset()
                basePID.setpoint = target.siValue
            }
        }

    /**
     * The PID Constants control exactly how the PID Controller attempts to reach the target.
     *
     * @see PIDConstants
     */
    public var constants: PIDConstants
        get() = basePID.constants
        set(pidConstants) {
            if (pidConstants != basePID.constants) {
                basePID.reset()
                basePID.constants = pidConstants
            }
        }

    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    public val error: Quantity<I>
        get() = Quantity(getInput().siValue - basePID.setpoint)
}