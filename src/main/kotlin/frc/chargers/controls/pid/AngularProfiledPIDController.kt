package frc.chargers.controls.pid

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ProfiledPIDController
import frc.chargers.commands.RunCommand
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile



public class AngularProfiledPIDController(
    pidConstants: PIDConstants,
    private val getInput: () -> Angle,
    public val outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    public val continuousInputRange: ClosedRange<Angle>? = null,
    public val integralRange: ClosedRange<Voltage> = outputRange,
    target: Angle,
    constraints: AngularTrapezoidProfile.Constraints,
    private val feedforward: AngularMotorFF = AngularMotorFF.None,
    /**
     * Determines if the [UnitSuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    selfSustain: Boolean = false
) : FeedbackController<Angle, Voltage> {


    init{
        if(selfSustain){
            RunCommand{
                calculateOutput()
            }.schedule()
        }
    }


    private val pidController = ProfiledPIDController(0.0, 0.0, 0.0,constraints.inUnit(radians,seconds))
        .apply {
            constants = pidConstants
            goal = AngularTrapezoidProfile.State(target,AngularVelocity(0.0)).inUnit(radians,seconds)
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
    public override fun calculateOutput(): Voltage {
        val output = Voltage(pidController.calculate(getInput().inUnit(radians))) +
                feedforward.calculate(pidController.setpoint.velocity.ofUnit(radians/seconds))
        return ensureInOutputRange(output)
    }


    private fun ensureInOutputRange(output: Voltage): Voltage {
        return output.coerceIn(outputRange)
    }

    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    override var target: Angle
        get() = Quantity(pidController.goal.position)
        set(target) {
            if (target.siValue != pidController.goal.position) {
                pidController.reset(getInput().inUnit(radians))
                pidController.goal = AngularTrapezoidProfile.State(target,AngularVelocity(0.0)).inUnit(radians,seconds)
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
                pidController.reset(getInput().inUnit(radians))
                pidController.constants = pidConstants
            }
        }

    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    override val error: Angle
        get() = Angle(getInput().siValue - pidController.setpoint.position)
}