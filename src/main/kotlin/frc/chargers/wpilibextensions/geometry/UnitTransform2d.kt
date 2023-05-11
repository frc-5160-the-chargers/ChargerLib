package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d

public class UnitTransform2d{
    public val base: Transform2d

    public constructor(){
        base = Transform2d()
    }
    public constructor(initial: UnitPose2d, last: UnitPose2d){
        base = Transform2d(initial.base,last.base)
    }
    public constructor(translation: UnitTranslation2d, rotation: UnitRotation2d){
        base = Transform2d(translation.base,rotation.base)

    }

    public operator fun div(scalar: Double): UnitTransform2d = (base / scalar).inUnitForm()
    public operator fun times(scalar: Double): UnitTransform2d = (base * scalar).inUnitForm()
    public operator fun plus(other: UnitTransform2d): UnitTransform2d = (base + other.base).inUnitForm()

    public val x: Distance
        get() = base.x.meters
    public val y: Distance
        get() = base.y.meters
    public val translation: UnitTranslation2d
        get() = base.translation.inUnitForm()
    public val rotation: UnitRotation2d
        get() = base.rotation.inUnitForm()
    public operator fun unaryMinus(): UnitTransform2d = base.inverse().inUnitForm()

    public override operator fun equals(other: Any?): Boolean = if (other !is UnitTransform2d){
            false
        }else{
            base == other.base
        }
    override fun hashCode(): Int = base.hashCode()
}