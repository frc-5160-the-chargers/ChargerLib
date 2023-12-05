package frc.chargers.wpilibextensions.geometry.threedimensional

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.utils.math.units.KmeasureUnit
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.littletonrobotics.junction.LogTable


public data class UnitPose3d(
    public val siValue: Pose3d = Pose3d()
): AdvantageKitLoggable<UnitPose3d> {
    public constructor(x: Distance, y: Distance, z: Distance, rotation: Rotation3d): this(
        Pose3d(x.siValue,y.siValue,z.siValue, rotation)
    )

    public constructor(translation: UnitTranslation3d,rotation: Rotation3d): this(
        Pose3d(translation.siValue,rotation)
    )

    public fun inUnit(unit: KmeasureUnit<DistanceDimension>): Pose3d =
        Pose3d(
            translation.inUnit(unit),
            rotation
        )

    public fun toPose2d(): UnitPose2d = UnitPose2d(siValue.toPose2d())

    public fun transformBy(other: UnitTransform3d): UnitPose3d = UnitPose3d(siValue.transformBy(other.siValue))


    public val x: Distance get() = Distance(siValue.x)
    public val y: Distance get() = Distance(siValue.y)
    public val z: Distance get() = Distance(siValue.z)

    public val translation: UnitTranslation3d
        get() = siValue.translation.ofUnit(meters)

    public val rotation: Rotation3d
        get() = siValue.rotation



    public operator fun plus(other: UnitTransform3d): UnitPose3d = UnitPose3d(siValue + other.siValue)
    public operator fun minus(other: UnitPose3d): UnitTransform3d = UnitTransform3d(siValue.minus(other.siValue))
    public operator fun times(scalar: Double): UnitPose3d = UnitPose3d(siValue * scalar)
    public operator fun div(scalar: Double): UnitPose3d = UnitPose3d(siValue / scalar)
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

    override fun getFromLog(table: LogTable, category: String): UnitPose3d = UnitPose3d(
        x = table.getDouble("$category/xMeters",0.0).ofUnit(meters),
        y = table.getDouble("$category/yMeters",0.0).ofUnit(meters),
        z = table.getDouble("$category/zMeters",0.0).ofUnit(meters),
        rotation = Rotation3d(
            table.getDouble("$category/rollRad",0.0),
            table.getDouble("$category/pitchRad",0.0),
            table.getDouble("$category/yawRad",0.0)
        )
    )

}