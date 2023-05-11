package frc.chargers.controls.pid

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.commands.RunCommand
import frc.chargers.controls.Controller
import frc.chargers.hardware.motorcontrol.ctre.falcon

/**
 * to-do: Add SuperVoltageController, UnitSuperVoltageController
 * add motion profiling
 */
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
    public var getInput: () -> Double,
    public val outputRange: ClosedRange<Double> = Double.NEGATIVE_INFINITY..Double.POSITIVE_INFINITY,
    public val integralRange: ClosedRange<Double> = outputRange,
    target: Double,
    public val outputExtender: OutputExtension = OutputExtension{_,_ -> 0.0},
    updatePeriodically: Boolean = false
): Controller<Double> {

    init{
        if(updatePeriodically){
            RunCommand{calculateOutput()}.schedule()
        }

    }

    private val basePID: PIDController = PIDController(0.0, 0.0, 0.0)
        .apply {
            constants = pidConstants
            setpoint = target
            setIntegratorRange(integralRange.start, integralRange.endInclusive)
        }

    public fun enableContinuousInput(bottomValue: Double, topValue: Double){
        basePID.enableContinuousInput(bottomValue,topValue)
    }

    public fun disableContinuousInput(){
        basePID.disableContinuousInput()
    }

    public fun modifyInputGetter(newInputGetter: () -> Double){
        getInput = newInputGetter
    }






    /**
     * Calculates the next calculated output value.
     * If updatePeriodically is set to false,
     * Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     * otherwise, nothing else needs to be done.
     */
    public override fun calculateOutput(): Double {
        val output = basePID.calculate(getInput()) + outputExtender.calculate(target,error)
        return ensureInOutputRange(output)
    }


    private fun ensureInOutputRange(output: Double): Double =  output.coerceIn(outputRange)


    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    public var target: Double
        get() = basePID.setpoint
        set(target) {
            if (target != basePID.setpoint) {
                basePID.reset()
                basePID.setpoint = target
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
    public val error: Double
        get() = getInput() - basePID.setpoint
}

