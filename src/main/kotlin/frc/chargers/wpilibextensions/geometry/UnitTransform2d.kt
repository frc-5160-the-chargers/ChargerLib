package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Transform2d


/**
 * A wrapper for WPILib's [Transform2d], adding in Unit support.
 */
public class UnitTransform2d(
    public val translation: UnitTranslation2d,
    public val rotation: Angle
){

    public constructor(): this(UnitTranslation2d(),Angle(0.0))

    public constructor(initial: UnitPose2d, last: UnitPose2d): this(
        (last.translation - initial.translation).rotateBy(-initial.rotation),
        last.rotation - initial.rotation
    )

    private val baseTransform = inUnit(meters)

    public fun inUnit(unit: Distance): Transform2d = Transform2d(
        translation.inUnit(unit),
        rotation.asRotation2d()
    )


    public operator fun div(scalar: Double): UnitTransform2d = (baseTransform / scalar).ofUnit(meters)
    public operator fun times(scalar: Double): UnitTransform2d = (baseTransform * scalar).ofUnit(meters)
    public operator fun plus(other: UnitTransform2d): UnitTransform2d = (baseTransform + other.inUnit(meters)).ofUnit(meters)

    public val x: Distance
        get() = baseTransform.x.meters
    public val y: Distance
        get() = baseTransform.y.meters
    public operator fun unaryMinus(): UnitTransform2d = baseTransform.inverse().ofUnit(meters)

    public override operator fun equals(other: Any?): Boolean = if (other !is UnitTransform2d){
        false
    }else{
        baseTransform == other.inUnit(meters)
    }
    override fun hashCode(): Int = baseTransform.hashCode()
}