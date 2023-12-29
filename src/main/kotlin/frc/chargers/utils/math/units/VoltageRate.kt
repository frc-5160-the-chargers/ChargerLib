package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*

public typealias VoltageRateDimension = Dimension<Mass1,Length2,TimeN4,CurrentN1>

public typealias VoltageRate = Quantity<VoltageRateDimension>