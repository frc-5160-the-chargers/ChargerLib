package frc.chargers.wpilibextensions.geometry.threedimensional

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.utils.math.units.KmeasureUnit
import org.littletonrobotics.junction.LogTable


public data class UnitTransform3d(
    public val siValue: Transform3d = Transform3d()
): AdvantageKitLoggable<UnitTransform3d> {

    public constructor(translation: UnitTranslation3d, rotation: Rotation3d): this(
        Transform3d(translation.siValue,rotation)
    )

    public fun inUnit(unit: KmeasureUnit<DistanceDimension>): Transform3d = Transform3d(
        translation.inUnit(unit),
        rotation
    )

    public val translation: UnitTranslation3d get() = UnitTranslation3d(siValue.translation)

    public val rotation: Rotation3d get() = siValue.rotation

    public val x: Distance get() = Distance(siValue.x)
    public val y: Distance get() = Distance(siValue.y)
    public val z: Distance get() = Distance(siValue.z)


    public operator fun div(scalar: Double): UnitTransform3d = UnitTransform3d(siValue / scalar)
    public operator fun times(scalar: Double): UnitTransform3d = UnitTransform3d(siValue * scalar)
    public operator fun plus(other: UnitTransform3d): UnitTransform3d = UnitTransform3d(siValue + other.siValue)

    override fun pushToLog(table: LogTable, category: String) {
        table.apply{
            put("$category/xMeters",x.inUnit(meters))
            put("$category/yMeters",y.inUnit(meters))
            put("$category/zMeters",z.inUnit(meters))

            put("$category/rollRad",rotation.x)
            put("$category/pitchRad",rotation.y)
            put("$category/yawRad",rotation.z)
        }
    }

    override fun getFromLog(table: LogTable, category: String): UnitTransform3d = UnitTransform3d(
        translation = UnitTranslation3d(
            x = table.getDouble("$category/xMeters",0.0).ofUnit(meters),
            y = table.getDouble("$category/yMeters",0.0).ofUnit(meters),
            z = table.getDouble("$category/zMeters",0.0).ofUnit(meters)
        ),
        rotation = Rotation3d(
            table.getDouble("$category/rollRad",0.0),
            table.getDouble("$category/pitchRad",0.0),
            table.getDouble("$category/yawRad",0.0)
        )
    )

}