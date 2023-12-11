package frc.chargers.hardware.sensors.cameras.vision.photonvision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.wpilibj.RobotBase.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.cameras.vision.*
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget

/**
 * A wrapper over PhotonVision's [PhotonCamera] that is designed to detect apriltags.
 */
public class ChargerPhotonCam(
    name: String,
    public val lensHeight: Distance,
    public val mountAngle: Angle
): PhotonCamera(name){

    public inner class ApriltagPipeline(
        override val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        private val logInputs: LoggableInputsProvider
    ): VisionPipeline<VisionResult.AprilTag>{

        init{
            reset()
            if (isSimulation()){
                Alert.warning(text =
                    "You have instantiated a Real Photon Cam pipeline in sim. " +
                    "This class still supports log replay in sim; " +
                    "however, it is reccomended to use VisionSim instead."
                ).active = true
            }
        }

        override fun reset(){
            pipelineIndex = index
            println("Photon Camera with name $name has had it's pipeline reset to $index")
        }

        override val lensHeight: Distance = this@ChargerPhotonCam.lensHeight
        override val mountAngle: Angle = this@ChargerPhotonCam.mountAngle

        override val visionData: VisionData<VisionResult.AprilTag>?
            by logInputs.nullableValue(
                default = emptyAprilTagVisionData()
            ){
                val data = latestResult

                val bestTarget = data.bestTarget
                val otherTargets = data.getTargets()
                otherTargets.remove(bestTarget)

                return@nullableValue if (!data.hasTargets() || isSimulation()) null else VisionData(
                    data.timestampSeconds.ofUnit(seconds),
                    toAprilTagTarget(bestTarget),
                    otherTargets.map{toAprilTagTarget(it)}
                )
            }

        public inner class PoseEstimator(
            robotToCamera: UnitTransform3d,
            fieldTags: AprilTagFieldLayout,
            strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        ): RobotPoseSupplier, PhotonPoseEstimator(
            fieldTags,
            strategy,
            this@ChargerPhotonCam,
            robotToCamera.inUnit(meters)
        ){

            init{
                setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
            }

            override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default

            override val robotPoseMeasurement: Measurement<UnitPose2d>?
                by logInputs.nullableValue(
                    default = Measurement(UnitPose2d(),0.0.seconds)
                ){
                    val signal = update()
                    return@nullableValue if (signal.isEmpty || isSimulation()){
                        null
                    }else{
                        Measurement(
                            UnitPose2d(signal.get().estimatedPose.toPose2d()),
                            signal.get().timestampSeconds.ofUnit(seconds)
                        )
                    }
                }
        }
    }

    public inner class ColorPipeline(
        override val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): VisionPipeline<VisionResult.Generic>{

        init{
            if (isSimulation()){
                Alert.warning(text =
                "You have instantiated a Real Photon Cam pipeline in sim. " +
                        "This class still supports log replay in sim; " +
                        "however, it is reccomended to use VisionSim instead."
                ).active = true
            }
        }

        override fun reset(){
            pipelineIndex = index
            println("Photon Camera with name $name has had it's pipeline reset to $index")
        }

        override val visionData: VisionData<VisionResult.Generic>?
            by logInputs.nullableValue(
                default = emptyGenericVisionData()
            ){
                val data = latestResult

                val bestTarget = data.bestTarget
                val otherTargets = data.getTargets()
                otherTargets.remove(bestTarget)

                return@nullableValue if (!data.hasTargets() || isSimulation()){
                    null
                } else {
                    VisionData(
                        data.timestampSeconds.ofUnit(seconds),
                        toGenericTarget(bestTarget),
                        otherTargets.map{toGenericTarget(it)}
                    )
                }
            }


        override val lensHeight: Distance = this@ChargerPhotonCam.lensHeight
        override val mountAngle: Angle = this@ChargerPhotonCam.mountAngle

    }


    private fun toAprilTagTarget(target: PhotonTrackedTarget) =
        VisionResult.AprilTag(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            id = target.fiducialId,
            targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
        )

    private fun toGenericTarget(target: PhotonTrackedTarget) =
        VisionResult.Generic(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area
        )

}