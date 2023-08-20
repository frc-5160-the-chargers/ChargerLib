package frc.chargers.controls.pid

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ProfiledPIDController
import frc.chargers.commands.RunCommand
import frc.chargers.controls.Controller
import frc.chargers.controls.feedforward.LinearMotorFF
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile



public class LinearProfiledPIDController(
    pidConstants: PIDConstants,
    private val getInput: () -> Distance,
    public val outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    public val integralRange: ClosedRange<Voltage> = outputRange,
    target: Distance,
    constraints: LinearTrapezoidProfile.Constraints,
    private val feedforward: LinearMotorFF,
    /**
     * Determines if the [UnitSuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    private val selfSustain: Boolean = false
) : Controller<Voltage> {


    init{
        if(selfSustain){
            RunCommand{
                calculateOutput()
            }.schedule()
        }
    }


    private val pidController = ProfiledPIDController(0.0, 0.0, 0.0,constraints.inUnit(meters,seconds))
        .apply {
            constants = pidConstants
            goal = LinearTrapezoidProfile.State(target,Velocity(0.0)).inUnit(meters,seconds)
            setIntegratorRange(integralRange.start.siValue, integralRange.endInclusive.siValue)
        }

    /**
     * Calculates the next calculated output value. Should be called periodically, likely in [edu.wpi.first.wpilibj2.command.Command.execute]
     */
    public override fun calculateOutput(): Voltage {
        val output = Voltage(pidController.calculate(getInput().inUnit(meters))) +
                feedforward.calculate(pidController.setpoint.velocity.ofUnit(meters/seconds))
        return ensureInOutputRange(output)
    }


    private fun ensureInOutputRange(output: Voltage): Voltage {
        return output.coerceIn(outputRange)
    }

    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    public var target: Distance
        get() = Quantity(pidController.goal.position)
        set(target) {
            if (target.siValue != pidController.goal.position) {
                pidController.reset(getInput().inUnit(meters))
                pidController.goal = LinearTrapezoidProfile.State(target,Velocity(0.0)).inUnit(meters,seconds)
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
                pidController.reset(getInput().inUnit(meters))
                pidController.constants = pidConstants
            }
        }

    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    public val error: Distance
        get() = Distance(getInput().siValue - pidController.setpoint.position)
}