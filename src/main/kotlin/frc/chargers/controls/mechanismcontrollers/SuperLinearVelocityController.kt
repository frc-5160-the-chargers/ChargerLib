package frc.chargers.controls.mechanismcontrollers

import com.batterystaple.kmeasure.dimensions.VelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.plus
import edu.wpi.first.math.controller.PIDController
import frc.chargers.commands.RunCommand
import frc.chargers.controls.Controller
import frc.chargers.controls.feedforward.LinearFeedForward
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitOutputExtension
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.controls.pid.constants

public class SuperLinearVelocityController(
    pidConstants: PIDConstants,
    private var getInput: () -> Velocity,
    outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    integralRange: ClosedRange<Voltage> = outputRange,
    target: Velocity,
    public val feedforward: LinearFeedForward,
    updatePeriodically: Boolean = false,
    outputExtender: UnitOutputExtension<VelocityDimension,VoltageDimension> = UnitOutputExtension { _, _ -> Quantity(0.0) }
): Controller<Voltage>, UnitSuperPIDController<VelocityDimension, VoltageDimension>(
    pidConstants, getInput, outputRange, integralRange, target, updatePeriodically = updatePeriodically, outputExtender = outputExtender
) {
    override fun calculateOutput(): Quantity<VoltageDimension> {
        return super.calculateOutput() + feedforward.calculate(getInput())
    }

}