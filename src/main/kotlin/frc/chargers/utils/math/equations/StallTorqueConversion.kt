package frc.chargers.utils.math.equations

import com.batterystaple.kmeasure.dimensions.EnergyDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.quantities.times


@Suppress("MagicNumber")
public val stallTorqueToVoltage: UnitPolynomial<EnergyDimension,VoltageDimension> =
    UnitPolynomial(
        newtons * meters to volts,
        // Determined empirically; polynomial fit is arbitrary
        0.0, 6.20254, -4.08666, 1.33192, -0.0981478
    )