package frc.chargers.hardware.sensors.cameras.vision

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import org.photonvision.PhotonUtils
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * Represents a generic vision system that can detect one type of target and compile data about it.
 *
 * In a multi-pipeline vision system, this interface should represent 1 pipeline.
 */
public interface VisionPipeline<R: VisionResult> {

    /**
     * The pipeline index.
     */
    public val index: Int

    /**
     * resets the camera that the [VisionPipeline] belongs to in order to return proper results.
     */
    public fun reset(){}


    /**
     * Fetches the full vision data of the [VisionPipeline]; These are all from the exact same timestamp.
     */
    public val visionData: NonLoggableVisionData<R>?

    /**
     * Fetches the current best target of the [VisionPipeline].
     *
     * The values fetched here are not nessecarily from the exact same timestamp.
     */
    public val bestTarget: R?
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

