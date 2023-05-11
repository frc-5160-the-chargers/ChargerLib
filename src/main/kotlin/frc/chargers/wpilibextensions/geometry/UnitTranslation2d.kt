package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Translation2d

public class UnitTranslation2d(private val distance: Distance, public val rotation: UnitRotation2d){
    public constructor(distance: Distance, angle: Angle): this(distance,UnitRotation2d(angle))

    public val base: Translation2d = Translation2d(distance.inUnit(meters),rotation.base)

    public val x: Distance = base.x.meters
    public val y: Distance = base.y.meters
    public val norm: Distance = distance
    public val angle: Angle = rotation.angle




    public operator fun div(scalar: Double): UnitTranslation2d = (base/scalar).inUnitForm()
    public operator fun times(scalar: Double): UnitTranslation2d = (base*scalar).inUnitForm()
    public operator fun plus(other: UnitTranslation2d): UnitTranslation2d = (base + other.base).inUnitForm()
    public operator fun unaryMinus(): UnitTranslation2d = (-base).inUnitForm()


    public fun getDistance(other: UnitTranslation2d): Distance = base.getDistance(other.base).meters
    public fun interpolate(other: UnitTranslation2d, t: Double): UnitTranslation2d = base.interpolate(other.base,t).inUnitForm()
    public fun nearest(vararg poses: UnitTranslation2d): UnitTranslation2d = base.nearest(poses.toMutableList().map{it.base}).inUnitForm()






}