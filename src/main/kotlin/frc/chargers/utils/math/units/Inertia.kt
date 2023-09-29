package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*

public typealias InertiaDimension = MassAreaDimension

public typealias Inertia = MassArea


@JvmName("MassLengthTimesLength")
public operator fun MassLength.times(other: Length): Inertia = Inertia(
    siValue * other.siValue
)

@JvmName("MassTimesArea")
public operator fun Mass.times(other: Area): Inertia = Inertia(
    siValue * other.siValue
)


