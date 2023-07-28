package frc.chargers.controls.feedforward


import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.meters


public class LinearMotorFF(
    public val kS: Voltage,
    public val kV: Double,
    public val kA: Double,
    public val gravity: Gravity,
    public val distanceUnit: Distance,
    public val timeUnit: Time = seconds,
    public val getAcceleration: () -> Acceleration
): Feedforward<Velocity,Voltage>{

    override fun calculate(value: Velocity): Voltage =
        SimpleMotorFeedforward(kS.inUnit(volts),kV,kA)
            .calculate(
                value.inUnit(distanceUnit/ timeUnit),
                getAcceleration().inUnit(distanceUnit/ timeUnit / timeUnit)
            ).ofUnit(volts) + gravity.getValue()

    public fun convertToAngular(distancePerRadian: Distance): AngularMotorFF{
        val radiansPerMeter: Angle = (1 /distancePerRadian.inUnit(meters) ).ofUnit(radians)

        // note: siValue is used here because for kA, the units are too long for kmeasure to handle
        val standardizedKV = (kV.volts/ 1.ofUnit(distanceUnit/timeUnit)).siValue
        val standardizedKA = (kA.volts/ 1.ofUnit(distanceUnit/timeUnit/timeUnit)).siValue

        val newKV = standardizedKV / radiansPerMeter.inUnit(radians)
        val newKA = standardizedKA / radiansPerMeter.inUnit(radians)

        return AngularMotorFF(
            kS = kS,
            kV = newKV,
            kA = newKA,
            gravity = gravity,
            angleUnit = radians,
            timeUnit = seconds,
            getAcceleration = {getAcceleration() * (radiansPerMeter / 1.meters)}
        )
    }

    public fun convertToAngular(gearRatio: Double, wheelDiameter: Length): AngularMotorFF =
        convertToAngular(gearRatio*wheelDiameter)


}