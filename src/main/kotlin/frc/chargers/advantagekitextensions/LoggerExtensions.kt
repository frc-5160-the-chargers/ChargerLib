package frc.chargers.advantagekitextensions

import builddata.ChargerLibBuildConstants
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.Logger

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





