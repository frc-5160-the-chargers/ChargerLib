package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*

public typealias VoltageRateDimension = Dimension<Mass1,Length2,TimeN4,CurrentN1>

public typealias VoltageRate = KmeasureUnit<VoltageRateDimension>



public typealias VoltagePerAngleDimension = ElectricalPotentialDimension

public typealias VoltagePerAngle = KmeasureUnit<VoltagePerAngleDimension>



public typealias TorqueDimension = EnergyDimension

public typealias Torque = Energy



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