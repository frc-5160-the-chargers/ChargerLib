package frc.chargers.hardware.sensors.visionRedo

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import org.photonvision.PhotonUtils
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * Represents a generic vision system that can detect one type of target and compile data about it.
 *
 * In a multi-pipeline vision system, this interface should represent 1 pipeline.
 */
public interface VisionSystem<T: VisionTarget> {

    /**
     * Fetches the full vision data of the [VisionSystem]; These are all from the exact same timestamp.
     */
    public val visionData: VisionData<T>?

    /**
     * Fetches the current best target of the [VisionSystem].
     *
     * The values fetched here are not nessecarily from the exact same timestamp.
     */
    public val bestTarget: T?
        get() = visionData?.bestTarget


    /**
     * How high the vision camera's lens is from the ground.
     */
    public val lensHeight: Distance

    /**
     * The mount angle describes how many degrees the vision camera is from perfectly vertical.
     */
    public val mountAngle: Angle

    public fun horizontalDistanceToTarget(
        targetHeight: Distance, targetPitch: Angle = Angle(0.0)
    ): Distance = PhotonUtils.calculateDistanceToTargetMeters(
        lensHeight.inUnit(meters),
        targetHeight.inUnit(meters),
        mountAngle.inUnit(radians),
        targetPitch.inUnit(radians)
    ).ofUnit(meters)

    public fun diagonalDistanceToTarget(targetHeight: Distance): Distance =
        Distance(
            sqrt(horizontalDistanceToTarget(targetHeight).siValue.pow(2) + targetHeight.siValue.pow(2.0))
        )

}

public data class VisionData<out T: VisionTarget>(
    val timestamp: Time,
    val bestTarget: T,
    val otherTargets: List<T>
){
    public constructor(
        timestamp: Time,
        bestTarget: T,
        vararg otherTargets: T
    ): this(timestamp, bestTarget, listOf(*otherTargets))

    public val allTargets: List<T> = otherTargets + listOf(bestTarget)



}


public sealed class VisionTarget(
    public val tx: Double,
    public val ty: Double,
    public val areaPercent: Double
){

    public class Generic(
        tx: Double,
        ty: Double,
        areaPercent: Double
    ): VisionTarget(tx,ty,areaPercent)


    public class Apriltag(
        tx: Double,
        ty: Double,
        areaPercent: Double,
        public val id: Int,
        public val targetTransformFromCam: UnitTransform3d
    ): VisionTarget(tx,ty,areaPercent)

    public class ML(
        tx: Double,
        ty: Double,
        areaPercent: Double,
        public val identifier: Any
    ): VisionTarget(tx,ty,areaPercent)

}
