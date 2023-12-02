package frc.chargers.hardware.sensors.visionRedo

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import frc.chargers.hardware.sensors.RobotPoseSupplier
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
    @JvmField public val name: String,
    override val lensHeight: Distance,
    override val mountAngle: Angle
): VisionPipeline<VisionResult.Apriltag>, PhotonCamera(name){


    public inner class PoseEstimator(
        robotToCamera: UnitTransform3d,
        strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP,
        fieldTags: AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
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
            get(){
                val signal = update()
                return if (signal.isEmpty){
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


    override val visionData: VisionData<VisionResult.Apriltag>?
        get(){
            val data = latestResult
            if (!data.hasTargets()) return null

            val bestTarget = data.bestTarget
            val otherTargets = data.getTargets()
            otherTargets.remove(bestTarget)

            return VisionData(
                data.timestampSeconds.ofUnit(seconds),
                toVisionTarget(bestTarget),
                otherTargets.map{toVisionTarget(it)}
            )
        }


    private fun toVisionTarget(target: PhotonTrackedTarget) =
        VisionResult.Apriltag(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            id = target.fiducialId,
            targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
        )
}