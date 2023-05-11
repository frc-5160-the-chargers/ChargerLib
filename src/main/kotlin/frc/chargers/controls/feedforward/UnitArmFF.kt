package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward

public class UnitArmFF(
    public val inputUnit: Quantity<AngleDimension>,
    public val ks: Voltage,
    public val kg: Voltage,
    public val kv: Double,
    public val ka: Double
): AngularFeedForward{

    private val baseFeedForward: ArmFeedforward = ArmFeedforward(ks.inUnit(volts),kg.inUnit(volts),kv,ka)
    override fun calculate(
        currentAngle: Angle,
        velocityTarget: AngularVelocity,
        accelerationTarget: AngularAcceleration
    ): Voltage = baseFeedForward.calculate(currentAngle.inUnit(inputUnit),velocityTarget.inUnit(inputUnit/ seconds),
        accelerationTarget.inUnit(inputUnit/ seconds / seconds)).volts

    override fun calculate(
        currentAngle: Angle,
        velocityTarget: AngularVelocity
    ): Voltage = baseFeedForward.calculate(currentAngle.inUnit(inputUnit),velocityTarget.inUnit(inputUnit/ seconds)).volts


}