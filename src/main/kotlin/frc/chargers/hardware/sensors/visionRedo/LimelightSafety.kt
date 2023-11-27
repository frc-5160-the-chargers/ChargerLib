package frc.chargers.hardware.sensors.visionRedo

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

public fun requireLimelight(limelightName: String = "limelight"){
    if (limelightName !in allReqManagers.map{it.limelightName}){
        allReqManagers.add(LimelightRequirementManager(limelightName))
    }else{
        error("A limelight has somehow been required twice. This is an issue with ChargerLib.")
    }

}


internal val allReqManagers = mutableListOf<LimelightRequirementManager>()

internal class LimelightRequirementManager(val limelightName: String): SubsystemBase(){

    val hasGlobalRequirement: Boolean = false
    val hasCommandRequirement: Boolean = false

    fun reportError(){
        error("A limelight with the name " + limelightName + "has already been required by another class.")
    }
}


