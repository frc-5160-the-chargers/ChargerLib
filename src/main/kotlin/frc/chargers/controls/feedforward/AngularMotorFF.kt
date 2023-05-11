package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.controller.SimpleMotorFeedforward

public class AngularMotorFF(
    public val inputUnit: Quantity<AngleDimension>,
    public val ks: Voltage,
    public val kv: Double,
    public val ka: Double = 0.0
): AngularFeedForward{

    private val baseFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(ks.inUnit(volts),kv,ka)
    override fun calculate(
        currentAngle: Angle,
        velocityTarget: AngularVelocity,
        accelerationTarget: AngularAcceleration
    ): Voltage = baseFeedForward.calculate(velocityTarget.inUnit(inputUnit/seconds),
        accelerationTarget.inUnit(inputUnit/seconds/seconds)).volts

    override fun calculate(
        currentAngle: Angle,
        velocityTarget: AngularVelocity
    ): Voltage = baseFeedForward.calculate(velocityTarget.inUnit(inputUnit/seconds)).volts


}