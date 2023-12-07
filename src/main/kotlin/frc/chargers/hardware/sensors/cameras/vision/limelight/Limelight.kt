package frc.chargers.hardware.sensors.cameras.vision.limelight

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargerlibexternal.utils.LimelightHelpers.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.CommandBuilder
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.cameras.vision.*
import frc.chargers.utils.NullableMeasurement
import frc.chargers.utils.RequirementManager
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import kotlin.jvm.optionals.getOrNull


public class Limelight(
    @JvmField public val name: String = "limelight",
    public val lensHeight: Distance,
    public val mountAngle: Angle,
    public val defaultDriverStationIfUnavailable: DriverStation.Alliance = DriverStation.Alliance.Blue
){

    init{
        if (name !in allReqManagers.map{it.name}){
            allReqManagers.add(RequirementManager(name))
        }else{
            error("This limelight shares a name with another limelight.")
        }

        if (isSimulation()){
            Alert.warning(text =
            "You have instantiated a Photon camera in sim. " +
                    "This class still supports log replay in sim; " +
                    "however, it is reccomended to use VisionSim instead."
            ).active = true
        }
    }



    public companion object{
        private val allReqManagers = mutableListOf<RequirementManager>()
        private val limelightRequiredButNotInstantiatedAlert =
            Alert.warning(text = "A limelight was required; however, the respective object was never instantiated. ")
        public fun requireIndefinitely(llName: String = "limelight"){
            for (reqManager in allReqManagers){
                if (reqManager.name == llName){
                    reqManager.requireIndefinetly()
                    return
                }
            }
            limelightRequiredButNotInstantiatedAlert.active = true
        }

        context(Command)
        public fun requireTemporarily(llName: String = "limelight"){
            for (reqManager in allReqManagers){
                if (reqManager.name == llName){
                    reqManager.requireTemporarily()
                    return
                }
            }
            limelightRequiredButNotInstantiatedAlert.active = true
        }

        context(CommandBuilder)
        public fun requireTemporarily(llName: String = "limelight"){
            for (reqManager in allReqManagers){
                if (reqManager.name == llName){
                    reqManager.requireTemporarily()
                    return
                }
            }
            limelightRequiredButNotInstantiatedAlert.active = true
        }

        public fun removeIndefiniteRequirement(llName: String = "limelight"){
            for (reqManager in allReqManagers){
                if (reqManager.name == llName){
                    reqManager.removeRequirement()
                    return
                }
            }
            limelightRequiredButNotInstantiatedAlert.active = true
        }

    }




    /**
     * Represents a generic Limelight Pipeline, with a specific ID.
     */
    public interface Pipeline{
        public val id: Int
    }

    private val allPipelines: MutableList<Pipeline> = mutableListOf()




    public open inner class ApriltagPipeline(
        @SuppressWarnings
        public val logInputs: LoggableInputsProvider,
        final override val id: Int
    ): Pipeline, VisionPipeline<VisionResult.AprilTag> {
        init{
            if (id < 0 || id > 9){
                error("Your pipeline's ID is out of range.")
            }

            for (pipeline in allPipelines){
                if (id == pipeline.id){
                    error("A pipeline was initialized with the same ID as another.")
                }
            }

            // safe; id is initialized before being added.
            allPipelines.add(this as Pipeline)
        }


        public inner class PoseEstimator(
            @SuppressWarnings
            public val logInputs: LoggableInputsProvider,
        ): RobotPoseSupplier {
            override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default
            override val robotPoseMeasurement: NullableMeasurement<UnitPose2d>
                by logInputs.nullableValueMeasurement(
                    default = UnitPose2d()
                ){
                    val allianceColor = DriverStation.getAlliance().getOrNull()
                    val poseArray = when(allianceColor){
                        DriverStation.Alliance.Blue -> getBotPose_wpiBlue(name)
                        DriverStation.Alliance.Red -> getBotPose_wpiRed(name)

                        null -> when(defaultDriverStationIfUnavailable){
                            DriverStation.Alliance.Blue -> getBotPose_wpiBlue(name)
                            DriverStation.Alliance.Red -> getBotPose_wpiRed(name)
                        }
                    }

                    // return value
                    NullableMeasurement(
                        nullableValue = if (getTV(name) && isReal()){
                            UnitPose2d(
                                poseArray[0].ofUnit(meters),
                                poseArray[1].ofUnit(meters),
                                poseArray[5].ofUnit(degrees)
                            )
                        }else{
                            null
                        },
                        timestamp = fpgaTimestamp() - poseArray[6].ofUnit(milli.seconds)
                    )
                }
        }

        override fun reset(){
            setPipelineIndex(name,id)
        }


        override val visionData: VisionData<VisionResult.AprilTag>?
            by logInputs.nullableValue(
                default = emptyAprilTagVisionData()
            ){
                val completeData = getLatestResults(name).targetingResults

                val allTargets = completeData.targets_Fiducials.toVisionTargets()
                val bestTarget = allTargets[0]
                allTargets.removeAt(0)

                // return value
                if (!completeData.valid || isSimulation()) {
                    null
                }else if (getCurrentPipelineIndex(name).toInt() != id){
                    println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                    null
                }else{
                    VisionData(
                        completeData.timestamp_RIOFPGA_capture.ofUnit(seconds),
                        bestTarget, allTargets
                    )
                }
            }


        override val lensHeight: Distance = this@Limelight.lensHeight
        override val mountAngle: Angle = this@Limelight.mountAngle


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

    public inner class MLDetectorPipeline(logInputs: LoggableInputsProvider, id: Int): Pipeline, VisionPipeline<VisionResult.ML>, MLClassifierPipeline(logInputs, id){

        override fun reset(){
            setPipelineIndex(name,id)
        }

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
                val completeData = getLatestResults(name).targetingResults


                val allTargets = completeData.targets_Detector.toVisionTargets()
                val bestTarget = allTargets[0]
                allTargets.removeAt(0)

                if (!completeData.valid || isSimulation()) {
                    null
                }else if (getCurrentPipelineIndex(name).toInt() != id){
                    println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                    null
                }else{
                    VisionData(
                        completeData.timestamp_RIOFPGA_capture.ofUnit(seconds),
                        bestTarget, allTargets
                    )
                }
            }


        override val lensHeight: Distance = this@Limelight.lensHeight
        override val mountAngle: Angle = this@Limelight.mountAngle
    }

    public open inner class MLClassifierPipeline(
        @SuppressWarnings
        public val logInputs: LoggableInputsProvider,
        final override val id: Int
    ): Classifier<Int?>, Pipeline {
        init{
            if (id < 0 || id > 9){
                error("Your pipeline's ID is out of range.")
            }

            for (pipeline in allPipelines){
                if (id == pipeline.id){
                    error("A pipeline was initialized with the same ID as another.")
                }
            }

            // safe; id is initialized before being added.
            @Suppress("LeakingThis")
            allPipelines.add(this as Pipeline)
            setPipelineIndex(name,id)
        }

        override fun reset(){
            setPipelineIndex(name,id)
        }


        override val itemType: Int?
            by logInputs.nullableInt{
                if (getCurrentPipelineIndex(name).toInt() == id && getTV(name) && isReal()){
                    getNeuralClassID(name).toInt()
                }else{
                    null
                }
            }

    }




}