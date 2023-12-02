package frc.chargers.hardware.sensors.cameras.vision

import com.batterystaple.kmeasure.quantities.Time
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d

public data class VisionData<out R: VisionResult>(
    val timestamp: Time,
    val bestTarget: R,
    val otherTargets: List<R>
){
    public constructor(
        timestamp: Time,
        bestTarget: R,
        vararg otherTargets: R
    ): this(timestamp, bestTarget, listOf(*otherTargets))

    public val allTargets: List<R> = otherTargets + listOf(bestTarget)

}


public sealed class VisionResult(
    public val tx: Double,
    public val ty: Double,
    public val areaPercent: Double
){

    public class Generic(
        tx: Double,
        ty: Double,
        areaPercent: Double
    ): VisionResult(tx,ty,areaPercent)


    public class Apriltag(
        tx: Double,
        ty: Double,
        areaPercent: Double,
        public val id: Int,
        public val targetTransformFromCam: UnitTransform3d
    ): VisionResult(tx,ty,areaPercent)

    public class ML(
        tx: Double,
        ty: Double,
        areaPercent: Double,
        public val id: Int
    ): VisionResult(tx,ty,areaPercent)

}