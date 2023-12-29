package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.dimensions.MagneticFluxDimension
import com.batterystaple.kmeasure.quantities.*

/*
   These units below represent Feedforward kA and kV gains.
 */

/**
 * Represents a kV gain for an angular velocity targeting feedforward.
 */
public typealias VoltagePerAngularVelocity = Quantity<VoltagePerAngularVelocityDimension>

/**
 * Represents a kV gain for a linear velocity targeting feedforward.
 */
public typealias VoltagePerVelocity = Quantity<VoltagePerVelocityDimension>

/**
 * Represents a kA gain for an angular velocity targeting feedforward.
 */
public typealias VoltagePerAngularAcceleration = Quantity<VoltagePerAngularAccelerationDimension>

/**
 * Represents a kA gain for a linear velocity targeting feedforward.
 */
public typealias VoltagePerAcceleration = Quantity<VoltagePerAccelerationDimension>


public typealias VoltagePerAngularVelocityDimension = MagneticFluxDimension

public typealias VoltagePerVelocityDimension = Dimension<Mass1, Length1, TimeN2, CurrentN1>

public typealias VoltagePerAngularAccelerationDimension = Dimension<Mass1, Length2, TimeN1, CurrentN1>

public typealias VoltagePerAccelerationDimension = Dimension<Mass1, Length1, TimeN1, CurrentN1>

