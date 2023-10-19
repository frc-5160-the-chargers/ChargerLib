package frc.chargers.framework

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.*
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.chargers.builddata.ChargerLibBuildConstants
import frc.chargers.wpilibextensions.Alert
import org.littletonrobotics.junction.LoggedRobot
import java.nio.file.Files
import java.nio.file.Path


/**
 * A base class for an FRC robot, acting as a wrapper around AdvantageKit's [LoggedRobot].
 *
 * This is intended to be used within the command-based framework.
 */
public open class ChargerRobot(
    private val getRobotContainer: () -> ChargerRobotContainer,
    private val gitData: GitData,
    private val isReplay: Boolean,
    private val extraLoggerConfig: Logger.() -> Unit = {}
): LoggedRobot(){
    public companion object{
        private val periodicRunnables: MutableList<() -> Unit> = mutableListOf()

        private val simPeriodicRunnables: MutableList<() -> Unit> = mutableListOf()

        public fun addToPeriodicLoop(runnable: () -> Unit){
            periodicRunnables.add(runnable)
        }


        public fun addToSimPeriodicLoop(runnable: () -> Unit){
            simPeriodicRunnables.add(runnable)
        }
    }

    private lateinit var robotContainer: ChargerRobotContainer
    private lateinit var autonomousCommand: Command
    private lateinit var testCommand: Command


    private val noUsbSignalAlert = Alert.warning(text = "No logging to WPILOG is happening; cannot find USB stick")
    private val logReceiverQueueAlert = Alert.error(text = "Logging queue exceeded capacity, data will NOT be logged.")


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {


        val logger = Logger.getInstance()
        setUseTiming(
            RobotBase.isReal() || !isReplay
        )

        logger.apply{
            recordMetadata(
                "Robot", if (RobotBase.isReal()) "REAL" else if (isReplay) "REPLAY" else "SIM"
            )
            recordMetadata("ProjectName", gitData.projectName)
            recordMetadata("BuildDate", gitData.buildDate)
            recordMetadata("GitSHA", gitData.sha)
            recordMetadata("GitBranch", gitData.branch)
            when(gitData.dirty){
                0 -> recordMetadata("GitDirty", "All changes committed")
                1 -> recordMetadata("GitDirty", "Uncommitted changes")
                else -> recordMetadata("GitDirty", "Unknown")
            }

            recordMetadata("ChargerLibBuildDate", ChargerLibBuildConstants.BUILD_DATE)
            recordMetadata("ChargerLibGitSHA", ChargerLibBuildConstants.GIT_SHA)
            recordMetadata("ChargerLibGitBranch", ChargerLibBuildConstants.GIT_BRANCH)
            when(ChargerLibBuildConstants.DIRTY){
                0 -> recordMetadata("ChargerLibGitDirty", "All changes committed")
                1 -> recordMetadata("ChargerLibGitDirty", "Uncommitted changes")
                else -> recordMetadata("ChargerLibGitDirty", "Unknown")
            }

            // real robot
            if (RobotBase.isReal()){
                if (Files.exists(Path.of("media/sda1"))){
                    addDataReceiver(WPILOGWriter("media/sda1"))
                }else if (Files.exists(Path.of("media/sda2"))){
                    addDataReceiver(WPILOGWriter("media/sda2"))
                }else{
                    noUsbSignalAlert.active = true
                }
                addDataReceiver(NTSafePublisher())
            }else if (isReplay){
                // replay mode; sim
                val path = LogFileUtil.findReplayLog()
                setReplaySource(WPILOGReader(path))
                addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")))
            }else{
                // sim mode
                logger.addDataReceiver(NTSafePublisher())
                // maybe add DriverStationSim? idk
            }

            extraLoggerConfig()

            // no more configuration from this point on
            start()
        }

        LiveWindow.disableAllTelemetry()

        // inits robotContainer
        robotContainer = getRobotContainer()

        // custom extension function in chargerlib
        CommandScheduler.getInstance().apply{
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
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        periodicRunnables.forEach{
            it()
        }
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
        Logger.getInstance().apply{
            recordOutput("RemainingRamMB", Runtime.getRuntime().freeMemory() / 1024 / 1024)
            logReceiverQueueAlert.active = receiverQueueFault
        }
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
        robotContainer.disabledInit()
    }
    override fun disabledPeriodic() {
        robotContainer.disabledPeriodic()
    }

    /** This autonomous runs the autonomous command selected by your RobotContainer class.  */
    override fun autonomousInit() {
        testCommand.cancel()
        autonomousCommand = robotContainer.autonomousCommand
        autonomousCommand.schedule()
        robotContainer.autonomousInit()
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
        robotContainer.autonomousPeriodic()
    }
    override fun teleopInit() {
        autonomousCommand.cancel()
        testCommand.cancel()
        robotContainer.teleopInit()
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {
        robotContainer.teleopPeriodic()
    }
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
        testCommand = robotContainer.testCommand
        robotContainer.testInit()
        testCommand.schedule()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {
        robotContainer.testPeriodic()
    }

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {
        robotContainer.simulationInit()
    }

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
        robotContainer.simulationPeriodic()
        simPeriodicRunnables.forEach{
            it()
        }
    }


}









