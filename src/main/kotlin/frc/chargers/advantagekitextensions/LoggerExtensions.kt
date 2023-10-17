package frc.chargers.advantagekitextensions

import frc.chargers.builddata.ChargerLibBuildConstants
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.chargers.wpilibextensions.Alert
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.nio.file.Files
import java.nio.file.Path

public fun CommandScheduler.startCommandLog(){
    onCommandInitialize{
        Logger.getInstance().recordOutput("/ActiveCommands/${it.name}", true)
    }

    onCommandFinish {
        Logger.getInstance().recordOutput("/ActiveCommands/${it.name}", false)
    }

    onCommandInterrupt {
        Logger.getInstance().recordOutput("/ActiveCommands/${it.name}", false)
    }
}

private val noUsbSignalAlert by lazy{
    Alert.warning(text = "No logging to WPILOG is happening; cannot find USB stick")
}
public fun Logger.addRioUSBReceiver(){
    if (Files.exists(Path.of("media/sda1"))){
        addDataReceiver(WPILOGWriter("media/sda1"))
    }else if (Files.exists(Path.of("media/sda2"))){
        addDataReceiver(WPILOGWriter("media/sda2"))
    }else{
        noUsbSignalAlert.active = true
    }
}

public fun Logger.logGitDirty(value: Int){
    when(value){
        0 -> recordMetadata("GitDirty", "All changes committed")
        1 -> recordMetadata("GitDirty", "Uncommitted changes")
        else -> recordMetadata("GitDirty", "Unknown")
    }
}

/**
 * A convenience function to log ChargerLib-based metadata.
 *
 * @see ChargerLibBuildConstants
 */
public fun Logger.logChargerLibMetadata(){
    recordMetadata("ChargerLibBuildDate", ChargerLibBuildConstants.BUILD_DATE)
    recordMetadata("ChargerLibGitSHA", ChargerLibBuildConstants.GIT_SHA)
    recordMetadata("ChargerLibGitBranch", ChargerLibBuildConstants.GIT_BRANCH)
    when(ChargerLibBuildConstants.DIRTY){
        0 -> recordMetadata("ChargerLibGitDirty", "All changes committed")
        1 -> recordMetadata("ChargerLibGitDirty", "Uncommitted changes")
        else -> recordMetadata("ChargerLibGitDirty", "Unknown")
    }
}





