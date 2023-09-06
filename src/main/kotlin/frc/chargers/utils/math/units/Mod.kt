package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

public operator fun <D: AnyDimension> Quantity<D>.rem(other: Quantity<D>): Quantity<D> = Quantity(siValue%other.siValue)