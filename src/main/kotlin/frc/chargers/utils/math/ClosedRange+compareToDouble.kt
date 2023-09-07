package frc.chargers.utils.math

public operator fun <T : Comparable<T>> ClosedRange<T>.compareTo(value: T): Int = when {
    value < this.start -> -1
    value > this.endInclusive -> 1
    else -> 0
}

public operator fun <T : Comparable<T>> T.compareTo(range: ClosedRange<T>): Int = when {
    this < range.start -> -1
    this > range.endInclusive -> 1
    else -> 0
}