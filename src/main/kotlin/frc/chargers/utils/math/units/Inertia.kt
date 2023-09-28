package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*

public typealias InertiaDimension = Dimension<Mass1,Length2, Time0,Current0>

public typealias Inertia = Quantity<InertiaDimension>

@JvmName("MassLengthTimesLength")
public operator fun MassLength.times(other: Length): Inertia = Inertia(
    siValue * other.siValue
)

@JvmName("MassTimesArea")
public operator fun Mass.times(other: Area): Inertia = Inertia(
    siValue * other.siValue
)

