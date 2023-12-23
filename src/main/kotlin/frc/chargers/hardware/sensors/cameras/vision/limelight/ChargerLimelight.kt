package frc.chargers.hardware.sensors.cameras.vision.limelight

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.*
import frc.chargerlibexternal.limelight.LimelightHelpers.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.cameras.vision.*
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import kotlin.jvm.optionals.getOrNull


/**
 * A wrapper around the Limelight, optimized for usage within ChargerLib.
 */
public class ChargerLimelight(
    @JvmField public val name: String = "limelight",
    /**
     * Determines whether to fetch the full JSON dump from the limelight every loop.
     * If the json dump is not fetched, direct NetworkTables data is used instead;
     * this prevents the camera from detecting multiple targets.
     */
    useJsonDump: Boolean = false,
    public val lensHeight: Distance,
    public val mountAngle: Angle,
    public val defaultDriverStationIfUnavailable: DriverStation.Alliance = DriverStation.Alliance.Blue,
){
    // Limelight Helpers results
    private var latestResults: Results = Results().apply{ valid = false }

    private fun updateJson(){
        if (useJsonDump && isReal() && getTV(name)){
            latestResults = getLatestResults(name).targetingResults
        }else{
            latestResults.valid = false
        }
    }

    private fun hasTargets() = if (useJsonDump) latestResults.valid else getTV(name)

    public var useJsonDump: Boolean = false
        set(shouldEnable){
            if (shouldEnable){
                ChargerRobot.runPeriodically(addToFront = true, ::updateJson)
            }else{
                ChargerRobot.removeFromLoop(::updateJson)
                latestResults.valid = false
            }
            field = shouldEnable
        }

    init{
        this.useJsonDump = useJsonDump
    }


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

        override fun reset(){
            setPipelineIndex(name,index)
            println("Limelight with name $name has had it's pipeline reset to $index")
        }


        override val visionData: VisionData<VisionResult.AprilTag>?
            by logInputs.nullableValue(
                default = emptyAprilTagVisionData()
            ){
                if (isSimulation()) return@nullableValue null

                val allTargets: MutableList<VisionResult.AprilTag> = if (useJsonDump){
                    latestResults.targets_Fiducials.toVisionTargets()
                }else{
                    mutableListOf(
                        VisionResult.AprilTag(
                            getTX(name),
                            getTY(name),
                            getTA(name),
                            getFiducialID(name).toInt(),
                            getTargetPose3d_CameraSpace(name).ofUnit(meters) - UnitPose3d()
                        )
                    )
                }

                val bestTarget = allTargets.removeAt(0)

                return@nullableValue if (!hasTargets()) {
                    null
                }else if (getCurrentPipelineIndex(name).toInt() != index){
                    println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                    null
                }else{
                    VisionData(
                        latestResults.timestamp_RIOFPGA_capture.ofUnit(seconds),
                        bestTarget, allTargets
                    )
                }
            }

        override val lensHeight: Distance = this@ChargerLimelight.lensHeight

        override val mountAngle: Angle = this@ChargerLimelight.mountAngle

        public inner class PoseEstimator: RobotPoseSupplier {

            override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default

            override val robotPoseMeasurement: Measurement<UnitPose2d>?
                by logInputs.nullableValue(default = Measurement(UnitPose2d(), 0.0.seconds)){
                    if (isSimulation()) {
                        return@nullableValue null
                    }

                    val allianceColor =
                        DriverStation.getAlliance().getOrNull() ?: defaultDriverStationIfUnavailable

                    val poseArray: DoubleArray

                    when(allianceColor) {
                        DriverStation.Alliance.Blue -> {
                            poseArray = if (useJsonDump){
                                latestResults.botpose_wpiblue
                            }else{
                                getBotPose_wpiBlue(name)
                            }
                        }

                        DriverStation.Alliance.Red -> {
                            poseArray = if (useJsonDump){
                                latestResults.botpose_wpired
                            }else{
                                getBotPose_wpiRed(name)
                            }
                        }
                    }

                    return@nullableValue if (!hasTargets()) {
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

        override val visionData: VisionData<VisionResult.ML>?
            by logInputs.nullableValue(
                default = emptyMLVisionData()
            ){
                if (isSimulation()) return@nullableValue null

                val allTargets = if (useJsonDump){
                    latestResults.targets_Detector.toVisionTargets()
                }else{
                    mutableListOf(
                        VisionResult.ML(
                            getTX(name),
                            getTY(name),
                            getTA(name),
                            getNeuralClassID(name).toInt()
                        )
                    )
                }

                val bestTarget = allTargets.removeAt(0)

                return@nullableValue if (!hasTargets()) {
                    null
                }else if (getCurrentPipelineIndex(name).toInt() != index){
                    println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                    null
                }else{
                    VisionData(
                        latestResults.timestamp_RIOFPGA_capture.ofUnit(seconds),
                        bestTarget, allTargets
                    )
                }
            }

        override val lensHeight: Distance = this@ChargerLimelight.lensHeight

        override val mountAngle: Angle = this@ChargerLimelight.mountAngle

        private fun Array<LimelightTarget_Detector>.toVisionTargets(): MutableList<VisionResult.ML> =
            this.map{
                VisionResult.ML(
                    tx = it.tx,
                    ty = it.ty,
                    areaPercent = it.ta,
                    id = it.classID.toInt()
                )
            }.toMutableList()
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

                return@nullableInt if (
                    getCurrentPipelineIndex(name).toInt() == index && hasTargets()
                ){
                    if (useJsonDump){
                        latestResults.targets_Classifier[0].classID.toInt()
                    }else{
                        getNeuralClassID(name).toInt()
                    }
                }else{
                    null
                }
            }

    }




}