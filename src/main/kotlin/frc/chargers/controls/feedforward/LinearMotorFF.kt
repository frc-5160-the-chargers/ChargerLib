package frc.chargers.controls.feedforward


import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.*
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.meters


/**
 * A wrapper for WPILib's [SimpleMotorFeedforward], [ArmFeedforward]. and [ElevatorFeedforward],
 * which includes unit support for linear velocity inputs(i.e. meters/second).
 *
 * Note: the voltage unit here is not necessary,
 * as the voltage unit is always volts in SysID,
 * which is the only place where feedforwards are measured in FRC.
 */
public class LinearMotorFF(
    public val kS: Voltage,
    private val kV: Double,
    private val kA: Double,
    public val gravity: Gravity,
    public val distanceUnit: Distance,
    public val timeUnit: Time = seconds,
    public val getAcceleration: () -> Acceleration = {Acceleration(0.0)}
): Feedforward<Velocity,Voltage>{

    public companion object{
        public val None: LinearMotorFF = LinearMotorFF(0.0.volts,0.0,0.0,Gravity.None,meters)
    }

    public fun getKV(newDistanceUnit: Distance, newTimeUnit: Time): Double{
        return kV.ofUnit(volts * timeUnit / distanceUnit).inUnit(volts * newTimeUnit / newDistanceUnit)
    }

    /*
    For this function, we had to use some workarounds.
    
    Unfortunately, kmeasure does not support the combination of more than 3 units, so the unit that kA is supposed to be in (volts * seconds * seconds / meters)
    would result in a compiling error.
    
    Instead, we treat kA here as a Voltage divided by an acceleration, so we just convert the acceleration to the proper units, then return the right value.
     */
    public fun getKA(newDistanceUnit: Distance, newTimeUnit: Time): Double{
        val kANumerator = kA
        val kADenominator = 1.ofUnit(distanceUnit/timeUnit/timeUnit).inUnit(newDistanceUnit/newTimeUnit/newTimeUnit)
        return kANumerator/kADenominator
    }

    override fun calculate(value: Velocity): Voltage =
        SimpleMotorFeedforward(kS.inUnit(volts),kV,kA)
            .calculate(
                value.inUnit(distanceUnit/ timeUnit),
                getAcceleration().inUnit(distanceUnit/ timeUnit / timeUnit)
            ).ofUnit(volts) + gravity.getOutput()

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