package frc.chargers.hardware.sensors.visionRedo

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

public fun requireLimelight(limelightName: String = "limelight"){

}

private fun test(input: Command){

}




internal object LimelightSafety{





    context(LimelightSafety)
    class LimelightRepr(limelightName: String): SubsystemBase(){

        val hasGlobalRequirement: Boolean = false
        val hasCommandRequirement: Boolean = false
    }

    fun reportError(limelightName: String){
        error("A limelight with the name " + limelightName + "has already been required by another class.")
    }
}