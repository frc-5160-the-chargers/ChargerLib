package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.Interpolatable
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.rotation.asRotation2d
import org.littletonrobotics.junction.LogTable

/**
 * A wrapper for WPILib's [Translation2d], adding in Unit support.
 */
@JvmInline
public value class UnitTranslation2d(
    public val siValue: Translation2d = Translation2d()
): Interpolatable<UnitTranslation2d>, AdvantageKitLoggable<UnitTranslation2d> {

    public constructor(x: Distance, y: Distance): this(Translation2d(x.siValue,y.siValue))

    // a companion object must be used to prevent a platform declaration crash,
    // as Distance and Angle are both represented as Doubles during JVM runtime.
    public companion object{
        public operator fun invoke(norm: Distance, angle: Angle): UnitTranslation2d =
            UnitTranslation2d(
                Translation2d(norm.siValue,angle.asRotation2d())
            )
    }


    /**
     * The X distance component of the Translation.
     *
     * @see Translation2d.getX
     */
    public val x: Distance get() = Distance(siValue.x)

    /**
     * The Y distance component of the Translation.
     *
     * @see Translation2d.getY
     */
    public val y: Distance get() = Distance(siValue.y)

    /**
     * The distance this translation is from the origin.
     *
     * @see Translation2d.getNorm
     */
    public val norm: Distance get() = Distance(siValue.norm)


    public fun inUnit(unit: Distance): Translation2d = Translation2d(x.inUnit(unit),y.inUnit(unit))
    public operator fun minus(other: UnitTranslation2d): UnitTranslation2d = siValue.minus(other.inUnit(meters)).ofUnit(meters)
    public operator fun div(scalar: Double): UnitTranslation2d = (siValue/scalar).ofUnit(meters)
    public operator fun times(scalar: Double): UnitTranslation2d = (siValue*scalar).ofUnit(meters)
    public operator fun plus(other: UnitTranslation2d): UnitTranslation2d = (siValue + other.inUnit(meters)).ofUnit(meters)
    public operator fun unaryMinus(): UnitTranslation2d = (-siValue).ofUnit(meters)


    public fun getDistance(other: UnitTranslation2d): Distance = siValue.getDistance(other.inUnit(meters)).meters
    override fun interpolate(other: UnitTranslation2d, t: Double): UnitTranslation2d = siValue.interpolate(other.inUnit(meters),t).ofUnit(meters)

    public fun rotateBy(other: Angle): UnitTranslation2d = siValue.rotateBy(other.asRotation2d()).ofUnit(meters)
    override fun pushToLog(table: LogTable, category: String) {
        table.apply{
            put("$category/xMeters",x.inUnit(meters))
            put("$category/yMeters",y.inUnit(meters))
        }
    }

    override fun getFromLog(table: LogTable, category: String): UnitTranslation2d = UnitTranslation2d(
        table.getDouble("$category/xMeters",0.0).ofUnit(meters),
        table.getDouble("$category/yMeters",0.0).ofUnit(meters)
    )


}

