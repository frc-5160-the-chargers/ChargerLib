package frc.chargers.advantagekitextensions

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



