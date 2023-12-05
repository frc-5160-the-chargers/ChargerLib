package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.utils.math.units.KmeasureUnit
import frc.chargers.wpilibextensions.geometry.rotation.asAngle
import frc.chargers.wpilibextensions.geometry.rotation.asRotation2d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import org.littletonrobotics.junction.LogTable

/**
 * A wrapper around WPILib's [UnitPose2d] class, adding in units support.
 */
public data class UnitPose2d(
    public val siValue: Pose2d = Pose2d()
): AdvantageKitLoggable<UnitPose2d>{
    public constructor(translation: UnitTranslation2d, rotation: Angle): this(
        Pose2d(translation.siValue, rotation.asRotation2d())
    )

    public constructor(x: Distance, y: Distance, rotation: Angle): this(
        Pose2d(x.siValue,y.siValue,rotation.asRotation2d())
    )

    /**
     * The translation component of the pose.
     *
     * @see Pose2d.getTranslation
     */
    public val translation: UnitTranslation2d get() = UnitTranslation2d(siValue.translation)

    /**
     * The rotation component of the pose.
     *
     * @see Pose2d.getRotation
     */
    public val rotation: Angle get() = siValue.rotation.asAngle()

    /**
     * The X distance component of the pose.
     *
     * @see Pose2d.getX
     */
    public val x: Distance get() = Distance(siValue.x)

    /**
     * The y component of the pose.
     *
     * @see Pose2d.getY
     */
    public val y: Distance get() = Distance(siValue.y)

    /**
     * Converts this pose to a [UnitPose3d].
     */
    public fun toPose3d(): UnitPose3d = UnitPose3d(Pose3d(siValue))


    /**
     * Converts this pose to a pose with a specific specified unit.
     */
    public fun inUnit(unit: KmeasureUnit<DistanceDimension>): Pose2d =
        Pose2d(translation.inUnit(unit),rotation.asRotation2d())

    public operator fun plus(other: UnitTransform2d): UnitPose2d = UnitPose2d(siValue + other.siValue)
    public operator fun minus(other: UnitPose2d): UnitTransform2d = UnitTransform2d(siValue - other.siValue)
    public operator fun times(scalar: Double): UnitPose2d = UnitPose2d(siValue * scalar)
    public operator fun div(scalar: Double): UnitPose2d = UnitPose2d(siValue / scalar)
    override fun pushToLog(table: LogTable, category: String) {
        table.apply{
            put("$category/xMeters",x.inUnit(meters))
            put("$category/yMeters",y.inUnit(meters))
            put("$category/rotationRad",rotation.inUnit(radians))
        }
    }

    override fun getFromLog(table: LogTable, category: String): UnitPose2d {
        return UnitPose2d(
            x = table.get("$category/xMeters",0.0).ofUnit(meters),
            y = table.get("$category/yMeters",0.0).ofUnit(meters),
            rotation = table.get("$category/rotationRad",0.0).ofUnit(radians)
        )
    }

}