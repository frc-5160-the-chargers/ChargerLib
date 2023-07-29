package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.*



/**
 * A wrapper for WPILib's [SimpleMotorFeedforward], [ArmFeedforward]. and [ElevatorFeedforward],
 * which includes unit support for angular velocity inputs(i.e. radians/second).
 *
 * Note: the voltage unit here is not necessary,
 * as the voltage unit is always volts in SysID,
 * which is the only place where feedforwards are measured in FRC.
 */
public class AngularMotorFF(
    public val kS: Voltage,
    public val kV: Double,
    public val kA: Double = 0.0,
    public val gravity: Gravity = Gravity.None,
    public val angleUnit: Angle,
    public val timeUnit: Time = seconds,
    public val getAcceleration: () -> AngularAcceleration = {AngularAcceleration(0.0)}
): Feedforward<AngularVelocity,Voltage>{

    override fun calculate(value: AngularVelocity): Voltage =
        SimpleMotorFeedforward(kS.inUnit(volts),kV,kA)
            .calculate(
                value.inUnit(angleUnit/timeUnit),
                getAcceleration().inUnit(angleUnit/timeUnit/timeUnit)
            ).ofUnit(volts) + gravity.getValue()

    public fun getAccelerationVoltage(): Voltage =
        (kA * getAcceleration().inUnit(angleUnit/timeUnit/timeUnit)).ofUnit(volts)

    public fun convertToLinear(distancePerRadian: Distance): LinearMotorFF{

        // note: siValue is used here because for kA, the units are too long for kmeasure to handle
        val standardizedKV = (kV.volts/1.ofUnit(angleUnit/timeUnit)).siValue
        val standardizedKA = (kA.volts/ 1.ofUnit(angleUnit/timeUnit/timeUnit)).siValue

        val newKV = standardizedKV / distancePerRadian.inUnit(meters)
        val newKA = standardizedKA / distancePerRadian.inUnit(meters)

        return LinearMotorFF(
            kS = kS,
            kV = newKV,
            kA = newKA,
            gravity = gravity,
            distanceUnit = meters,
            timeUnit = seconds,
            getAcceleration = {getAcceleration() * (distancePerRadian / 1.radians)}
        )
    }

    public fun convertToLinear(gearRatio: Double, wheelDiameter: Length): LinearMotorFF =
        convertToLinear(gearRatio*wheelDiameter)

}