package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.Interpolatable



public class UnitTranslation2d(
    public val x: Distance,
    public val y: Distance,
): Interpolatable<UnitTranslation2d>{

    public constructor(
        distance: Distance,
        angle: Angle
    ): this(distance * Scalar(cos(angle)), distance * Scalar(sin(angle)) )

    public constructor(): this(Distance(0.0),Distance(0.0))

    // simplifies calculation
    private val baseTranslation = inUnit(meters)
    public fun inUnit(unit: Distance): Translation2d = Translation2d(x.inUnit(unit),y.inUnit(unit))





    public operator fun minus(other: UnitTranslation2d): UnitTranslation2d = baseTranslation.minus(other.inUnit(meters)).ofUnit(meters)
    public operator fun div(scalar: Double): UnitTranslation2d = (baseTranslation/scalar).ofUnit(meters)
    public operator fun times(scalar: Double): UnitTranslation2d = (baseTranslation*scalar).ofUnit(meters)
    public operator fun plus(other: UnitTranslation2d): UnitTranslation2d = (baseTranslation + other.inUnit(meters)).ofUnit(meters)
    public operator fun unaryMinus(): UnitTranslation2d = (-baseTranslation).ofUnit(meters)


    public fun getDistance(other: UnitTranslation2d): Distance = baseTranslation.getDistance(other.inUnit(meters)).meters
    override fun interpolate(other: UnitTranslation2d, t: Double): UnitTranslation2d = baseTranslation.interpolate(other.inUnit(meters),t).ofUnit(meters)

    public fun rotateBy(other: Angle): UnitTranslation2d = baseTranslation.rotateBy(other.asRotation2d()).ofUnit(meters)

}