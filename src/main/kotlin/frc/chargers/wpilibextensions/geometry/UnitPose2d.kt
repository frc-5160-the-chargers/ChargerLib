package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d

public class UnitPose2d(public val x: Distance, public val y: Distance, public val rotation: UnitRotation2d){

    public val base: Pose2d = Pose2d(x.inUnit(meters),y.inUnit(meters),rotation.base)
    public constructor(x: Distance, y: Distance, rotation: Rotation2d): this(x,
        y,
        rotation.inUnitForm())

    public constructor(x: Distance, y: Distance, theta: Angle): this(x,
        y,
        UnitRotation2d(theta))

    public val theta: Angle = rotation.angle

    public operator fun plus(other: UnitTransform2d): UnitPose2d = base.plus(other.base).inUnitForm()
    public operator fun minus(other: UnitPose2d): UnitTransform2d = base.minus(other.base).inUnitForm()
    public operator fun times(scalar: Double): UnitPose2d = base.times(scalar).inUnitForm()
    public operator fun div(scalar: Double): UnitPose2d = base.div(scalar).inUnitForm()

    public fun nearest(vararg poses: UnitPose2d): UnitPose2d = base.nearest(poses.toMutableList().map{it.base}).inUnitForm()



}