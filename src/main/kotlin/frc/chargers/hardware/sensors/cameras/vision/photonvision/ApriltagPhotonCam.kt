package frc.chargers.hardware.sensors.cameras.vision.photonvision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.cameras.vision.*
import frc.chargers.utils.Measurement
import frc.chargers.utils.NullableMeasurement
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget

/**
 * A wrapper over PhotonVision's [PhotonCamera].
 */
public class ApriltagPhotonCam(
    /**
     * The namespace of which the ApriltagPhotonCam logs to:
     * Ensure that this namespace is the same accross real and sim equivalents of the photon camera.
     * @see LoggableInputsProvider
     */
    public val logNamespace: LoggableInputsProvider,
    override val lensHeight: Distance,
    override val mountAngle: Angle
): VisionPipeline<VisionResult.AprilTag>, PhotonCamera(logNamespace.logGroup){


    public inner class PoseEstimator(
        logNamespace: LoggableInputsProvider,
        robotToCamera: UnitTransform3d,
        fieldTags: AprilTagFieldLayout,
        strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP,
    ): RobotPoseSupplier, PhotonPoseEstimator(
        fieldTags,
        strategy,
        this,
        robotToCamera.inUnit(meters)
    ){

        init{
            setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        }


        override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default
        override val robotPoseMeasurement: NullableMeasurement<UnitPose2d>
            by logNamespace.timestampedNullableValue(
                logNullRepr = NullableMeasurement(UnitPose2d(), Time(0.0))
            ){
                val signal = update()
                if (signal.isEmpty){
                    NullableMeasurement(null, fpgaTimestamp())
                }else{
                    val data = signal.get()
                    Measurement(
                        UnitPose2d(data.estimatedPose.toPose2d()),
                        data.timestampSeconds.ofUnit(seconds)
                    )
                }
            }
    }


    override val visionData: VisionData<VisionResult.AprilTag>? by logNamespace.genericNullableValue(
        loggedNullRepr = emptyAprilTagVisionData()
    ){
        val data = latestResult

        val bestTarget = data.bestTarget
        val otherTargets = data.getTargets()
        otherTargets.remove(bestTarget)

        // return value
        if (!data.hasTargets()) null else VisionData(
            data.timestampSeconds.ofUnit(seconds),
            toVisionTarget(bestTarget),
            otherTargets.map{toVisionTarget(it)}
        )
    }


    private fun toVisionTarget(target: PhotonTrackedTarget) =
        VisionResult.AprilTag(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            id = target.fiducialId,
            targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
        )
}