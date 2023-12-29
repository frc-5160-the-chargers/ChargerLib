package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.interop.transformWithSIValue
import com.batterystaple.kmeasure.quantities.*
import kotlin.internal.LowPriorityInOverloadResolution
import kotlin.math.pow


public operator fun <D: AnyDimension> Quantity<D>.rem(other: Quantity<D>): Quantity<D> =
    Quantity(siValue % other.siValue)

public fun <D: AnyDimension> Quantity<D>.pow(exponent: Number): Quantity<D> =
    transformWithSIValue{it.pow(exponent.toDouble())}

public fun <D: AnyDimension> sqrt(value: Quantity<D>): Quantity<D> =
    Quantity(kotlin.math.sqrt(value.siValue))

@JvmName("MassLengthTimesLength")
public operator fun MassLength.times(other: Length): Inertia = Inertia(
    siValue * other.siValue
)

@JvmName("MassTimesArea")
public operator fun Mass.times(other: Area): Inertia = Inertia(
    siValue * other.siValue
)

@JvmName("KvMultipliedAngular")
@LowPriorityInOverloadResolution
public operator fun VoltagePerAngularVelocity.times(other: Time): VoltagePerAngularAcceleration =
    VoltagePerAngularAcceleration(siValue * other.siValue)

@JvmName("KvMultipliedLinear")
public operator fun VoltagePerVelocity.times(other: Time): VoltagePerAcceleration =
    VoltagePerAcceleration(siValue * other.siValue)


/**
 * Example usage:
 *
 * ```
 * val ka = Voltage(1.0) * Time(1.0) * Time(1.0) / Angle(1.0)
 * val ka = 1.0.ofUnit(volts * seconds * seconds / radians)
 * ```
 */
@JvmName("AngularKAConstantIdentity")
public operator fun Quantity<Dimension<Mass1, Length2, TimeN1, CurrentN1>>.div(other: Angle): VoltagePerAngularAcceleration =
    VoltagePerAngularAcceleration(siValue / other.siValue)


/**
 * Example usage:
 *
 * ```
 * val ka = Voltage(1.0) * Time(1.0) * Time(1.0) / Distance(1.0)
 * val ka = 1.0.ofUnit(volts * seconds * seconds / meters)
 * ```
 */
@JvmName("LinearKAConstantDerivation")
public operator fun Quantity<Dimension<Mass1, Length2, TimeN1, CurrentN1>>.div(other: Distance): VoltagePerAcceleration =
    VoltagePerAcceleration(siValue / other.siValue)


@JvmName("AngularKVConstantTimesAngularVelocity")
public operator fun VoltagePerAngularVelocity.times(other: AngularVelocity): Voltage =
    Voltage(siValue * other.siValue)

@JvmName("LinearKVConstantTimesVelocity")
public operator fun VoltagePerVelocity.times(other: Velocity): Voltage =
    Voltage(siValue * other.siValue)

@JvmName("LinearKVConstantTimeDistance")
public operator fun VoltagePerVelocity.times(other: Distance): VoltagePerAngularVelocity =
    VoltagePerAngularVelocity(siValue * other.siValue)


@JvmName("AngularKAConstantTimesAngularAcceleration")
public operator fun VoltagePerAngularAcceleration.times(other: AngularAcceleration): Voltage =
    Voltage(siValue * other.siValue)


@JvmName("LinearKVConstantTimesAcceleration")
public operator fun VoltagePerAcceleration.times(other: Acceleration): Voltage =
    Voltage(siValue * other.siValue)

@JvmName("LinearKAConstantTimeDistance")
public operator fun VoltagePerAcceleration.times(other: Distance): VoltagePerAngularAcceleration =
    VoltagePerAngularAcceleration(siValue * other.siValue)

