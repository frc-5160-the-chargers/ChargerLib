package frc.chargers.controls.pid

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ProfiledPIDController
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.feedforward.LinearMotorFF
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.geometry.motion.AngularTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints
import frc.chargers.wpilibextensions.geometry.motion.LinearTrapezoidProfile
import frc.chargers.wpilibextensions.geometry.ofUnit


/**
 * Creates a PID controller, coupled with motion profiling, which accepts a linear input.
 * Wraps WPILib's [ProfiledPIDController], adding builtin units and feedforward support.
 */
public class LinearProfiledPIDController(
    pidConstants: PIDConstants,
    private val getInput: () -> Distance,
    public val outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    public val continuousInputRange: ClosedRange<Distance>? = null,
    public val integralRange: ClosedRange<Voltage> = outputRange,
    target: Distance,
    constraints: LinearMotionConstraints,
    private val feedforward: LinearMotorFF = LinearMotorFF.None,
    /**
     * Determines if the [UnitSuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    selfSustain: Boolean = false
) : FeedbackController<Distance, Voltage> {


    init{
        if(selfSustain){
            ChargerRobot.addToPeriodicLoop(::calculateOutput)
        }
    }


    private fun Distance.standardize(): Distance =
        if (continuousInputRange == null) this else this.inputModulus(continuousInputRange)


    private val pidController = ProfiledPIDController(0.0, 0.0, 0.0,constraints.inUnit(meters,seconds))
        .apply {
            constants = pidConstants
            goal = LinearTrapezoidProfile.State(target,Velocity(0.0)).inUnit(meters,seconds)
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
            if (continuousInputRange != null){
                enableContinuousInput(
                    continuousInputRange.start.inUnit(meters),
                    continuousInputRange.endInclusive.inUnit(meters)
                )
            }
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    override fun calculateOutput(): Voltage {
        val output = Voltage(pidController.calculate(getInput().standardize().inUnit(meters))) +
                feedforward.calculate(pidController.setpoint.velocity.ofUnit(meters/seconds))
        return output.coerceIn(outputRange)
    }


    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    override var target: Distance
        get() = Quantity(pidController.goal.position)
        set(target) {
            if (target.siValue != pidController.goal.position) {
                pidController.goal = LinearTrapezoidProfile.State(target,Velocity(0.0)).inUnit(meters,seconds)
            }
        }

    /**
     * An alternative to the target variable; sets the profiled PID controller to a specific [AngularTrapezoidProfile.State].
     */
    public var targetState: LinearTrapezoidProfile.State
        get() = pidController.goal.ofUnit(meters,seconds)
        set(target){
            if (target != pidController.goal.ofUnit(meters,seconds)){
                pidController.goal = target.inUnit(meters,seconds)
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
                pidController.constants = pidConstants
            }
        }

    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    override val error: Distance
        get() = Distance(getInput().siValue - pidController.setpoint.position)
}