package frc.chargers.utils

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.Timer
import frc.chargers.hardware.sensors.cameras.Limelight
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.math.pow

public fun SwerveDrivePoseEstimator.setInitialPoseWithLimelights(limelights: List<Limelight>,
                                                                 pipelinesToTry: List<Int>,
                                                                 gyroRotation2d: Rotation2d,
                                                                 modulePositionArray: Array<SwerveModulePosition>): Boolean{
    for (limelight in limelights) {
        for (pipeline in pipelinesToTry){
            if(!limelight.inUse){
                limelight.pipeline = pipeline
                if (limelight.hasTarget) {
                    var centered_botpose = limelight.botpose

                    // checks which alliance the robot is in, using botpose.
                    // if the botpose value for 0 is perfectly 0, it's far more likely that the robot
                    // isn't reporting botpose.
                    // note: the robotCurrentAllianceColor is only changed if it is unknown,
                    // so that if there is a manual change somewhere else, this won't interfere with it.
                    if(robotCurrentAllianceColor == AllianceColor.UNKNOWN){
                        if(limelight.botpose[0] > 0.0){
                            robotCurrentAllianceColor = AllianceColor.BLUE
                        }else if (limelight.botpose[0] < 0.0){
                            robotCurrentAllianceColor = AllianceColor.RED
                        }
                    }

                    if(robotCurrentAllianceColor == AllianceColor.BLUE){
                        this.resetPosition(
                            gyroRotation2d,
                            modulePositionArray,
                            Pose2d(limelight.botpose_wpiblue[0],limelight.botpose_wpiblue[1], Rotation2d())
                        )
                    }else if (robotCurrentAllianceColor == AllianceColor.RED){
                        this.resetPosition(
                            gyroRotation2d,
                            modulePositionArray,
                            Pose2d(limelight.botpose_wpired[0],limelight.botpose_wpired[1], Rotation2d())
                        )
                    }

                    return true
                }
            }
        }
    }
    return false
}

public fun SwerveDrivePoseEstimator.setInitialPoseWithPhotonVision(cameras: List<PhotonCamera>,
                                                                   cameraTransformsFromRobotCenter: List<Transform3d>,
                                                                   fieldLayout: AprilTagFieldLayout,
                                                                   gyroRotation2d: Rotation2d,
                                                                   modulePositionArray: Array<SwerveModulePosition>): Boolean{
    var photonPoseEstimators = MutableList(cameras.size){i ->
        PhotonPoseEstimator(fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
            cameras[i],
            cameraTransformsFromRobotCenter[i])
    }
    photonPoseEstimators.forEach{estimator -> estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)}

    var resultList = List(cameras.size){i -> photonPoseEstimators[i].update()}

    for (result in resultList){
        if (result.isPresent){
            this.resetPosition(
                gyroRotation2d,
                modulePositionArray,
                result.get().estimatedPose.toPose2d()
            )
            return true
        }
    }
    return false
}


public fun SwerveDrivePoseEstimator.updateWithLimelights(limelights: List<Limelight>,
                                                         pipelinesToTry: List<Int>){
    var availableLimelights: MutableList<Limelight> = mutableListOf()

    for (limelight in limelights) {
        if (!limelight.inUse) {
            availableLimelights.add(limelight)
        }
    }
    if (availableLimelights.size > 0) {
        for (limelight in availableLimelights) {
            limelight.pipeline = pipelinesToTry[0]
            var tempCounter = 1
            while (!limelight.hasTarget && tempCounter < pipelinesToTry.size - 1) {
                limelight.pipeline = pipelinesToTry[tempCounter]
                tempCounter += 1
            }

            var numberOfResults = limelight.fiducialResults.size

            // scales based off of the number of apriltags detected.
            // note that these numbers are arbitrary lol; I just randomly decided them
            this.setVisionMeasurementStdDevs(
                VecBuilder.fill(
                    0.9 / (1.25.pow(numberOfResults)),
                    0.9 / (1.25.pow(numberOfResults)),
                    0.9 / (1.25.pow(numberOfResults))
                )
            )
            if(robotCurrentAllianceColor == AllianceColor.BLUE){
                this.addVisionMeasurement(
                    Pose2d(limelight.botpose_wpiblue[0], limelight.botpose_wpiblue[1], Rotation2d()),
                    Timer.getFPGATimestamp() - (limelight.tl / 1000.0) - (limelight.cl / 1000.0)
                )
            }else if(robotCurrentAllianceColor == AllianceColor.RED){
                this.addVisionMeasurement(
                    Pose2d(limelight.botpose_wpired[0], limelight.botpose_wpired[1], Rotation2d()),
                    Timer.getFPGATimestamp() - (limelight.tl / 1000.0) - (limelight.cl / 1000.0)
                )
            }


        }
    }
}




public fun SwerveDrivePoseEstimator.updateWithPhotonVision(cameras: List<PhotonCamera>,
                                                           cameraTransformsFromRobotCenter: List<Transform3d>,
                                                           fieldLayout: AprilTagFieldLayout
){
    var photonPoseEstimators = MutableList(cameras.size){i ->
        PhotonPoseEstimator(fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
            cameras[i],
            cameraTransformsFromRobotCenter[i])
    }
    photonPoseEstimators.forEach{estimator -> estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)}

    var resultList = List(cameras.size){i -> photonPoseEstimators[i].update()}

    var botposeList: MutableList<EstimatedRobotPose> = mutableListOf()

    for (result in resultList){
        if (result.isPresent){
            botposeList.add(result.get())
        }
    }

    this.setVisionMeasurementStdDevs(VecBuilder.fill(0.9,0.9,0.9))
    for (pose in botposeList){
        this.addVisionMeasurement(
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds
        )
    }
}