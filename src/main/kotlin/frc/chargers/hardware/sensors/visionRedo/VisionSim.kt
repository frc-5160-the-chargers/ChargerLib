package frc.chargers.hardware.sensors.visionRedo

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.utils.Measurement
import frc.chargers.utils.NullableMeasurement
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.rotation.zAngle
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.SimVisionSystem
import org.photonvision.SimVisionTarget
import org.photonvision.targeting.PhotonTrackedTarget



public class ApriltagCamSim (
    camName: String,
    private val robotPoseSupplier: RobotPoseSupplier,
    private val robotToCam: UnitTransform3d,
    fov: Angle,
    ledRange: Distance,
    minTargetArea: Double,
    cameraResWidth: Int,
    cameraResHeight: Int,
    private val fieldMap: AprilTagFieldLayout
): VisionPipeline<VisionResult.Apriltag> {
    private val camera = PhotonCamera(camName)
    private val simSystem = SimVisionSystem(
        camName,
        fov.inUnit(degrees),
        robotToCam.inUnit(meters),
        ledRange.inUnit(meters),
        cameraResWidth, cameraResHeight, minTargetArea
    ).apply{
        addVisionTargets(fieldMap)
    }
    private var previousRobotPose = UnitPose2d()

    init{
        ChargerRobot.runPeriodically{
            val pose = robotPoseSupplier.robotPose ?: previousRobotPose
            simSystem.processFrame(pose.inUnit(meters))
            previousRobotPose = pose
        }
    }

    override val visionData: VisionData<VisionResult.Apriltag>?
        get(){
            val data = camera.latestResult
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
    override val lensHeight: Distance = robotToCam.z
    override val mountAngle: Angle = robotToCam.rotation.zAngle

    private fun toVisionTarget(target: PhotonTrackedTarget) =
        VisionResult.Apriltag(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            id = target.fiducialId,
            targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
        )


    public inner class PoseEstimator(
        strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP,
    ): RobotPoseSupplier, PhotonPoseEstimator(
        fieldMap,
        strategy,
        this.camera,
        robotToCam.inUnit(meters)
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
}



public class MLCamSim(
    camName: String,
    private val robotPoseSupplier: RobotPoseSupplier,
    robotToCam: UnitTransform3d,
    fov: Angle,
    ledRange: Distance,
    minTargetArea: Double,
    cameraResWidth: Int,
    cameraResHeight: Int,
    vararg targets: Target
): VisionPipeline<VisionResult.ML>{
    public data class Target(
        val pose: UnitPose3d,
        val width: Distance,
        val height: Distance,
        val id: Int
    ){
        internal fun toPhotonTarget(): SimVisionTarget = SimVisionTarget(
            pose.inUnit(meters),
            width.inUnit(meters),
            height.inUnit(meters),
            id
        )
    }



    private val camera = PhotonCamera(camName)
    private val simSystem = SimVisionSystem(
        camName,
        fov.inUnit(degrees),
        robotToCam.inUnit(meters),
        ledRange.inUnit(meters),
        cameraResWidth, cameraResHeight, minTargetArea
    ).apply{
        targets.forEach{
            addSimVisionTarget(it.toPhotonTarget())
        }
    }
    private var previousRobotPose = UnitPose2d()

    init{
        ChargerRobot.runPeriodically{
            val pose = robotPoseSupplier.robotPose ?: previousRobotPose
            simSystem.processFrame(pose.inUnit(meters))
            previousRobotPose = pose
        }
    }

    override val visionData: VisionData<VisionResult.ML>?
        get(){
            val data = camera.latestResult
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
    override val lensHeight: Distance = robotToCam.z
    override val mountAngle: Angle = robotToCam.rotation.zAngle

    private fun toVisionTarget(target: PhotonTrackedTarget) =
        VisionResult.ML(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            id = target.fiducialId
        )

}


