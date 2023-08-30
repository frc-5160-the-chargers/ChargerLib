package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

public operator fun <D: AnyDimension> Quantity<D>.rem(other: Double): Quantity<D> = Quantity(siValue%other)

public operator fun <D: AnyDimension> Quantity<D>.rem(other: Int): Quantity<D> = Quantity(siValue%other)