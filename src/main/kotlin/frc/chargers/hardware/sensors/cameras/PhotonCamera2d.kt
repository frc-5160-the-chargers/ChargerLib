package frc.chargers.hardware.sensors.cameras

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonUtils
import kotlin.math.pow

/**
 * A drop-in replacement for photonCamera, which implements the VisionCamera interface(and has built-in pose estimation).
 */
public class PhotonCamera2d(
    name: String,
    lensHeight: Distance,
    mountAngle: Angle,
    fieldLayout: AprilTagFieldLayout,
    cameraTransform: Transform3d
): PhotonCamera(name), VisionCamera {

    public val poseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
        this,
        cameraTransform)
    init{
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }


    override val hasTarget: Boolean
        get() = latestResult.hasTargets()
    override val xAngularOffset: Angle
        get() = latestResult.bestTarget.yaw.degrees

    override val yAngularOffset: Angle
        get() = latestResult.bestTarget.pitch.degrees

    override val area: Double
        get() = latestResult.bestTarget.area

    override val xDistanceOffset: Distance
        get() = latestResult.bestTarget.bestCameraToTarget.x.meters

    override val yDistanceOffset: Distance
        get() = latestResult.bestTarget.bestCameraToTarget.y.meters

    override val zDistanceOffset: Distance
        get() = latestResult.bestTarget.bestCameraToTarget.z.meters

    override val poseValid: Boolean
        get() = poseEstimator.update().isPresent

    // note: TBD!
    override val poseReliability: Matrix<N3, N1> = VecBuilder.fill(0.8,0.8,0.8)

    override val wpiBotpose: Pose2d
        get() = poseEstimator.update().get().estimatedPose.toPose2d()

    override fun getDiagonalDistance(height: Distance): Distance {
        return (PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            height.inUnit(meters),
            CAMERA_PITCH_RADIANS,
            yAngularOffset.inUnit(radians)).pow(2.0) + height.inUnit(meters).pow(2.0)).meters
    }

    override fun getDistance(height: Distance): Distance {
        return PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            height.inUnit(meters),
            CAMERA_PITCH_RADIANS,
            yAngularOffset.inUnit(radians)).meters
    }

}