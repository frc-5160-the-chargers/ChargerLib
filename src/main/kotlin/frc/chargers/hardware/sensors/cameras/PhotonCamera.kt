package frc.chargers.hardware.sensors.cameras


import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.StandardDeviation
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonUtils

/**
 * A [PhotonCamera] wrapper, which implements the VisionCamera3d interface.
 */
public open class BasicPhotonCamera(
    name: String,
    public val lensHeight: Distance,
    public val mountAngle: Angle,
): PhotonCamera(name), VisionCamera3d {


    override val hasTarget: Boolean
        get() = latestResult.hasTargets()
    override val thetaX: Angle
        get() = latestResult.bestTarget.yaw.degrees

    override val thetaY: Angle
        get() = latestResult.bestTarget.pitch.degrees

    override val area: Double
        get() = latestResult.bestTarget.area

    override val xDistance: Distance
        get() = latestResult.bestTarget.bestCameraToTarget.x.meters

    override val yDistance: Distance
        get() = latestResult.bestTarget.bestCameraToTarget.y.meters

    override val zDistance: Distance
        get() = latestResult.bestTarget.bestCameraToTarget.z.meters

    override fun getDistance(height: Distance): Distance {
        return PhotonUtils.calculateDistanceToTargetMeters(
            lensHeight.inUnit(meters),
            height.inUnit(meters),
            mountAngle.inUnit(radians),
            thetaY.inUnit(radians)
        ).meters
    }

}

/**
 * A [PhotonCamera] which can supply the pose of the robot.
 *
 * @see BasicPhotonCamera
 */
public class OdometryPhotonCamera(
    name: String,
    fieldLayout: AprilTagFieldLayout,
    xTranslation: Distance,
    yTranslation: Distance,
    zTranslation: Distance,
    roll: Angle,
    pitch: Angle,
    yaw: Angle
): BasicPhotonCamera(name,zTranslation,pitch), RobotPoseSupplier{
    public val poseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
        this,
        Transform3d(
            Translation3d(xTranslation.inUnit(meters), yTranslation.inUnit(meters),zTranslation.inUnit(meters)),
            Rotation3d(roll.inUnit(radians), pitch.inUnit(radians), yaw.inUnit(radians))
        )
    )


    init{
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }

    override val poseStandardDeviation: StandardDeviation
        get() = StandardDeviation.Default
    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() {
            val m = poseEstimator.update()
            return Measurement(
                m.get().estimatedPose.toPose2d().ofUnit(meters),
                m.get().timestampSeconds.ofUnit(seconds),
                m.isEmpty
            )
        }

}

/*

    */