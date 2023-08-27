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
import frc.chargers.hardware.sensors.CameraPoseSupplier
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.StandardDeviation
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonUtils
import kotlin.math.pow

/**
 * A drop-in replacement for photonCamera, which implements the VisionCamera interface.
 */
public class BasicPhotonCamera(
    name: String,
    public val lensHeight: Distance,
    public val mountAngle: Angle,
): PhotonCamera(name), VisionCamera3d {

    /*
    public val poseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
        this,
        cameraTransform)


    init{
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }
    */



    private inner class poseProvider: RobotPoseSupplier, CameraPoseSupplier{
        override val cameraPoseMeasurement: Measurement<UnitPose2d>
            get() = TODO("Not yet implemented")
        override val poseStandardDeviation: StandardDeviation
            get() = TODO("Not yet implemented")
        override val robotPoseMeasurement: Measurement<UnitPose2d>
            get() = TODO("Not yet implemented")

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


    override fun getDiagonalDistance(height: Distance): Distance {
        return (PhotonUtils.calculateDistanceToTargetMeters(
            lensHeight.inUnit(meters),
            height.inUnit(meters),
            mountAngle.inUnit(radians),
            yAngularOffset.inUnit(radians)).pow(2.0) + height.inUnit(meters).pow(2.0)
                ).meters
    }

    override fun getDistance(height: Distance): Distance {
        return PhotonUtils.calculateDistanceToTargetMeters(
            lensHeight.inUnit(meters),
            height.inUnit(meters),
            mountAngle.inUnit(radians),
            yAngularOffset.inUnit(radians)
        ).meters
    }

}
