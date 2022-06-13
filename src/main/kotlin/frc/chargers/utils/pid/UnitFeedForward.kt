package frc.chargers.utils.pid

import com.batterystaple.kmeasure.Dimension
import com.batterystaple.kmeasure.DimensionedQuantity

public fun interface UnitFeedForward<I : Dimension, O : Dimension> {
    public fun calculate(target: DimensionedQuantity<I>, error: DimensionedQuantity<I>): DimensionedQuantity<O>
}