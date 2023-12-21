package frc.chargers.utils

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.commands.commandbuilder.CommandBuilder
import frc.chargers.wpilibextensions.Alert

/**
 * A utility class used for managing a requirable system within the command-based framework.
 *
 * This class should be used with classes that aren't subsystems,
 * but need to enforce safety within their use in commands and other subsystems.
 * One of the examples of this is the [frc.chargers.hardware.sensors.cameras.vision.limelight] class.
 */
public class RequirementManager(public val name: String) {
    public fun requireIndefinetly(){
        if (!isRequiredIndefinetly){
            isRequiredIndefinetly = true
        }else {
            error(name + "has been required in 2 different places.")
        }
    }


    context(Command)
    public fun requireTemporarily(){
        if (!isRequiredIndefinetly){
            addRequirements(dummySubsystem)
        }else {
            error(name + "has been required in 2 different places.")
        }
    }

    context(CommandBuilder)
    public fun requireTemporarily(){
        if (!isRequiredIndefinetly){
            addRequirements(dummySubsystem)
        }else {
            error(name + "has been required in 2 different places.")
        }
    }

    public fun removeRequirement(){
        if (isRequiredIndefinetly){
            isRequiredIndefinetly = false
            removeNonexistentRequirementWarning.active = false
        }else{
            removeNonexistentRequirementWarning.active = true
        }
    }




    private val dummySubsystem = object: SubsystemBase(){
        override fun getName(): String = this@RequirementManager.name
    }
    private var isRequiredIndefinetly = false
    private val removeNonexistentRequirementWarning = Alert.warning(
        text = "Requirements for " + name + "were removed when they were never set in the first place."
    )

}