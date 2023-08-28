package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Pose2d

/**
 * A wrapper for WPILib's [Pose2d], adding in Unit support.
 */
public data class UnitPose2d(public val translation: UnitTranslation2d, public val rotation: Angle){

    public constructor(x: Distance, y: Distance, rotation: Angle): this(UnitTranslation2d(x,y), rotation)

    public constructor(): this(UnitTranslation2d(),Angle(0.0))

    public val x: Distance = translation.x
    public val y: Distance = translation.y


    // used to simplify calculations
    private val basePose = inUnit(meters)
    public fun inUnit(unit: Distance): Pose2d = Pose2d(translation.inUnit(unit),rotation.asRotation2d())

    public operator fun plus(other: UnitTransform2d): UnitPose2d = basePose.plus(other.inUnit(meters)).ofUnit(meters)
    public operator fun minus(other: UnitPose2d): UnitTransform2d = basePose.minus(other.inUnit(meters)).ofUnit(meters)
    public operator fun times(scalar: Double): UnitPose2d = basePose.times(scalar).ofUnit(meters)
    public operator fun div(scalar: Double): UnitPose2d = basePose.div(scalar).ofUnit(meters)

}