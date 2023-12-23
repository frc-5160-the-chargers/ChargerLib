package frc.chargers.controls.pid

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints


/**
 * Creates a PID controller, coupled with motion profiling, which accepts an angular input.
 * Wraps WPILib's [ProfiledPIDController], adding builtin units and feedforward support.
 */
public class AngularProfiledPIDController(
    pidConstants: PIDConstants,
    private val getInput: () -> Angle,
    public val outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    public val continuousInputRange: ClosedRange<Angle>? = null,
    public val integralRange: ClosedRange<Voltage> = outputRange,
    target: Angle,
    constraints: AngularMotionConstraints,
    private val feedforward: AngularMotorFF = AngularMotorFF.None,
    /**
     * Determines if the [UnitSuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    selfSustain: Boolean = false
) : FeedbackController<Angle, Voltage> {


    init{
        if(selfSustain){
            ChargerRobot.runPeriodically(runnable = ::calculateOutput)
        }
    }


    private fun Angle.standardize(): Angle =
        if (continuousInputRange == null) this else this.inputModulus(continuousInputRange)


    private val pidController = ProfiledPIDController(0.0, 0.0, 0.0,constraints.inUnit(radians,seconds))
        .apply {
            constants = pidConstants
            goal = TrapezoidProfile.State(target.inUnit(radians),0.0)
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
            if (continuousInputRange != null){
                enableContinuousInput(
                    continuousInputRange.start.inUnit(radians),
                    continuousInputRange.endInclusive.inUnit(radians)
                )
            }
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    override fun calculateOutput(): Voltage {
        val output = Voltage(pidController.calculate(getInput().standardize().inUnit(radians))) +
                feedforward.calculate(pidController.setpoint.velocity.ofUnit(radians/seconds))
        return output.coerceIn(outputRange)
    }



    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    override var target: Angle
        get() = Quantity(pidController.goal.position)
        set(target) {
            if (target.siValue != pidController.goal.position) {
                pidController.goal = TrapezoidProfile.State(target.inUnit(radians),0.0)
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
    override val error: Angle
        get() = Angle(getInput().siValue - pidController.setpoint.position)
}