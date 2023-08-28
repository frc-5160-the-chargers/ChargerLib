package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Translation3d

/*
public data class UnitTranslation3d(
    public val x: Distance,
    public val y: Distance,
    public val z: Distance
) {

    private val baseTranslation = inUnit(meters)
    public constructor(): this(Distance(0.0),Distance(0.0),Distance(0.0))

    public fun inUnit(unit: Distance): Translation3d = Translation3d(
        x.inUnit(unit),
        y.inUnit(unit),
        z.inUnit(unit)
    )

    public operator fun div(scalar: Double): UnitTranslation3d = (baseTranslation/scalar).ofUnit(meters)

}

 */