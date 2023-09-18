package frc.chargers.utils.math
public fun Double.mapBetweenRanges(from: ClosedRange<Double>, to: ClosedRange<Double>): Double {
    require(this in from) { "Value must be within initial range." }

    val proportionIntoRange: Double = (this - from.start) / (from.endInclusive - from.start)
    val distanceIntoToRange: Double = proportionIntoRange * (to.endInclusive - to.start)
    return to.start + distanceIntoToRange
}