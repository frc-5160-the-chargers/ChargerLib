package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.SimpleMotorFeedforward



public class LinearMotorFF(
    public val inputUnit: Quantity<DistanceDimension>,
    public val ks: Voltage,
    public val kv: Double,
    public val ka: Double = 0.0
): LinearFeedForward{

    private val baseFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(ks.inUnit(volts),kv,ka)
    override fun calculate(
        velocityTarget: Velocity,
        accelerationTarget: Acceleration
    ): Voltage = baseFeedForward.calculate(velocityTarget.inUnit(inputUnit/ seconds),
        accelerationTarget.inUnit(inputUnit/ seconds / seconds)).volts

    override fun calculate(
        velocityTarget: Velocity
    ): Voltage = baseFeedForward.calculate(velocityTarget.inUnit(inputUnit/ seconds)).volts

}