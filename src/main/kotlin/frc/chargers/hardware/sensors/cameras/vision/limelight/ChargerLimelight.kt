package frc.chargers.hardware.sensors.cameras.vision.limelight

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.*
import frc.chargerlibexternal.limelight.LimelightHelpers.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.cameras.vision.*
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import kotlin.jvm.optionals.getOrNull


public class ChargerLimelight(
    @JvmField public val name: String = "limelight",
    public val lensHeight: Distance,
    public val mountAngle: Angle,
    public val defaultDriverStationIfUnavailable: DriverStation.Alliance = DriverStation.Alliance.Blue
){


    public inner class ApriltagPipeline(
        override val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        private val logInputs: LoggableInputsProvider
    ): VisionPipeline<VisionResult.AprilTag> {
        init{
            reset()
            if (index < 0 || index > 9){
                error("Your pipeline's ID is out of range.")
            }
        }


        public inner class PoseEstimator: RobotPoseSupplier {

            override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default

            override val robotPoseMeasurement: Measurement<UnitPose2d>?
                by logInputs.nullableValue(
                    default = Measurement(
                        UnitPose2d(),
                        0.0.seconds
                    )
                ){
                    if (isSimulation()) {
                        return@nullableValue null
                    }

                    val allianceColor = DriverStation.getAlliance().getOrNull()
                    val poseArray = when(allianceColor){
                        DriverStation.Alliance.Blue -> getBotPose_wpiBlue(name)
                        DriverStation.Alliance.Red -> getBotPose_wpiRed(name)

                        null -> when(defaultDriverStationIfUnavailable){
                            DriverStation.Alliance.Blue -> getBotPose_wpiBlue(name)
                            DriverStation.Alliance.Red -> getBotPose_wpiRed(name)
                        }
                    }

                    return@nullableValue if (!getTV(name) || isSimulation()) {
                        null
                    } else {
                        Measurement(
                            value = UnitPose2d(
                                poseArray[0].ofUnit(meters),
                                poseArray[1].ofUnit(meters),
                                poseArray[5].ofUnit(degrees)
                            ),
                            timestamp = fpgaTimestamp() - poseArray[6].ofUnit(milli.seconds)
                        )
                    }
                }
        }

        override fun reset(){
            setPipelineIndex(name,index)
            println("Limelight with name $name has had it's pipeline reset to $index")
        }


        override val visionData: VisionData<VisionResult.AprilTag>?
            by logInputs.nullableValue(
                default = emptyAprilTagVisionData()
            ){
                if (isSimulation()) return@nullableValue null

                val completeData = getLatestResults(name).targetingResults
                val allTargets = completeData.targets_Fiducials.toVisionTargets()
                val bestTarget = allTargets[0]
                allTargets.removeAt(0)

                return@nullableValue if (!completeData.valid) {
                    null
                }else if (getCurrentPipelineIndex(name).toInt() != index){
                    println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                    null
                }else{
                    VisionData(
                        completeData.timestamp_RIOFPGA_capture.ofUnit(seconds),
                        bestTarget, allTargets
                    )
                }
            }


        override val lensHeight: Distance = this@ChargerLimelight.lensHeight
        override val mountAngle: Angle = this@ChargerLimelight.mountAngle


        private fun Array<LimelightTarget_Fiducial>.toVisionTargets(): MutableList<VisionResult.AprilTag> =
            this.map{
                VisionResult.AprilTag(
                    tx = it.tx,
                    ty = it.ty,
                    areaPercent = it.ta,
                    id = it.fiducialID.toInt(),
                    // converts it to a UnitTransform3d.
                    targetTransformFromCam = it.targetPose_CameraSpace.ofUnit(meters) - UnitPose3d()
                )
            }.toMutableList()
    }

    public inner class MLDetectorPipeline(
        override val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): VisionPipeline<VisionResult.ML>, MLClassifierPipeline(index,logInputs){

        override fun reset(){ super<MLClassifierPipeline>.reset() }

        private fun Array<LimelightTarget_Detector>.toVisionTargets(): MutableList<VisionResult.ML> =
            this.map{
                VisionResult.ML(
                    tx = it.tx,
                    ty = it.ty,
                    areaPercent = it.ta,
                    id = it.classID.toInt()
                )
            }.toMutableList()

        override val visionData: VisionData<VisionResult.ML>?
            by logInputs.nullableValue(
                default = emptyMLVisionData()
            ){
                if (isSimulation()) return@nullableValue null
                
                val completeData = getLatestResults(name).targetingResults
                val allTargets = completeData.targets_Detector.toVisionTargets()
                val bestTarget = allTargets[0]
                allTargets.removeAt(0)

                return@nullableValue if (!completeData.valid) {
                    null
                }else if (getCurrentPipelineIndex(name).toInt() != index){
                    println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                    null
                }else{
                    VisionData(
                        completeData.timestamp_RIOFPGA_capture.ofUnit(seconds),
                        bestTarget, allTargets
                    )
                }
            }


        override val lensHeight: Distance = this@ChargerLimelight.lensHeight
        override val mountAngle: Angle = this@ChargerLimelight.mountAngle
    }

    public open inner class MLClassifierPipeline(
        private val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): Classifier<Int?> {
        init{
            if (index < 0 || index > 9){
                error("Your pipeline's ID is out of range.")
            }
            setPipelineIndex(name,index)
            println("Limelight with name $name has had it's pipeline reset to $index")
        }

        override fun reset(){
            setPipelineIndex(name,index)
            println("Limelight with name $name has had it's pipeline reset to $index")
        }


        override val itemType: Int?
            by logInputs.nullableInt{
                if (isSimulation()) return@nullableInt null
                if (getCurrentPipelineIndex(name).toInt() == index && getTV(name)){
                    getNeuralClassID(name).toInt()
                }else{
                    null
                }
            }

    }




}