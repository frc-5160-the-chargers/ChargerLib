package frc.chargers.hardware.sensors.cameras.vision.limelight

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.chargerlibexternal.utils.LimelightHelpers.*
import frc.chargers.commands.CommandBuilder
import frc.chargers.hardware.sensors.cameras.vision.Classifier
import frc.chargers.hardware.sensors.cameras.vision.NonLoggableVisionData
import frc.chargers.hardware.sensors.cameras.vision.VisionPipeline
import frc.chargers.hardware.sensors.cameras.vision.VisionResult
import frc.chargers.utils.RequirementManager
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d



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

        context(CommandBase)
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






    public inner class ApriltagPipeline(override val id: Int): Pipeline, VisionPipeline<VisionResult.AprilTag> {

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

        override fun reset(){
            setPipelineIndex(name,id)
        }


        override val visionData: NonLoggableVisionData<VisionResult.AprilTag>?
            get(){
                val completeData = getLatestResults(name).targetingResults
                if (!completeData.valid || getCurrentPipelineIndex(name).toInt() != id) {
                    println("pipeline is not correct; resetting pipeline of limelight")
                    return null
                }

                val allTargets = completeData.targets_Fiducials.toVisionTargets()
                val bestTarget = allTargets[0]
                allTargets.removeAt(0)

                return NonLoggableVisionData(
                    completeData.timestamp_RIOFPGA_capture.ofUnit(seconds),
                    bestTarget, allTargets
                )
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

    public inner class MLDetectorPipeline(id: Int): Pipeline, VisionPipeline<VisionResult.ML>, MLClassifierPipeline(id){

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

        override val visionData: NonLoggableVisionData<VisionResult.ML>?
            get(){


                val completeData = getLatestResults(name).targetingResults
                if (!completeData.valid || getCurrentPipelineIndex(name).toInt() != id) {
                    println("pipeline is not correct; resetting pipeline of limelight")
                    setPipelineIndex(name,id)
                    return null
                }

                val allTargets = completeData.targets_Detector.toVisionTargets()
                val bestTarget = allTargets[0]
                allTargets.removeAt(0)

                return NonLoggableVisionData(
                    completeData.timestamp_RIOFPGA_capture.ofUnit(seconds),
                    bestTarget, allTargets
                )
            }


        override val lensHeight: Distance = this@Limelight.lensHeight
        override val mountAngle: Angle = this@Limelight.mountAngle
    }

    public open inner class MLClassifierPipeline(final override val id: Int): Classifier<Int?>, Pipeline {
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
            setPipelineIndex(name,id)
        }


        override val itemType: Int?
            get(){
                return if (getCurrentPipelineIndex(name).toInt() == id && getTV(name)){
                    getNeuralClassID(name).toInt()
                }else{
                    setPipelineIndex(name,id)
                    null
                }

            }

    }




}