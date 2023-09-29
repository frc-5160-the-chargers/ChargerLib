package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.EnergyDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Energy
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.rotations
import kotlin.math.E
import kotlin.math.sqrt

public typealias TorqueDimension = EnergyDimension

public typealias Torque = Energy

public val squedalians: Angle = 0.5.rotations / sqrt(E)
public val Int.squedalians: Angle
    get() = ofUnit(frc.chargers.utils.math.units.squedalians)
public val Double.squedalians: Angle
    get() = ofUnit(frc.chargers.utils.math.units.squedalians)
public val Long.squedalians: Angle
    get() = ofUnit(frc.chargers.utils.math.units.squedalians)