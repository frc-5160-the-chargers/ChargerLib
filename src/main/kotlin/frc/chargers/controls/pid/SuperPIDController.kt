package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.SetpointSupplier
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.inputModulus

/**
 * Wraps WPILib's [PIDController], adding various improvements and units support.
 *
 * This class accepts Kmeasure [Quantity]s instead of [Double]s:
 * converting them to their siValue before and after the pid equation is applied.
 *
 * This ensures that the pid constants passed in do not need to change for different units;
 * they should be tuned with the respect to the siValue of the input and output.
 *
 * See [here](https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html) for an explanation of PID.
 */
public class SuperPIDController<I: AnyDimension, O: AnyDimension>(
    pidConstants: PIDConstants,
    private val getInput: () -> Quantity<I>,
    override var target: Quantity<I>,
    private val setpointSupplier: SetpointSupplier<Quantity<I>, Quantity<O>> = SetpointSupplier.Default(),
    private val continuousInputRange: ClosedRange<Quantity<I>>? = null,
    private val outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
    private val integralRange: ClosedRange<Quantity<O>> = outputRange,
    /**
     * Determines if the [SuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    selfSustain: Boolean = false,
) : FeedbackController<Quantity<I>, Quantity<O>> {
    private fun Quantity<I>.standardize(): Quantity<I> =
        if (continuousInputRange == null) this else this.inputModulus(continuousInputRange)

    init{
        if(selfSustain){
            ChargerRobot.runPeriodically(runnable = ::calculateOutput)
        }
    }


    private val pidController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
            if (continuousInputRange != null){
                enableContinuousInput(
                    continuousInputRange.start.siValue,
                    continuousInputRange.endInclusive.siValue
                )
            }
        }


    override fun calculateOutput(): Quantity<O> {
        val setpoint = setpointSupplier.getSetpoint(target)
        val pidOutput = Quantity<O>(
            pidController.calculate(
                getInput().standardize().siValue,
                setpoint.value.siValue
            )
        )
        val ffOutput = setpoint.feedforwardOutput
        return (pidOutput + ffOutput).coerceIn(outputRange)
    }


    override val error: Quantity<I>
        get() = Quantity(pidController.positionError)

}