package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.server.PathPlannerServer
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
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.utils.SparkMaxBurnManager
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
    private val config: RobotConfig
): LoggedRobot(config.loopPeriod.inUnit(seconds)){
    public companion object{
        private lateinit var burnManager: SparkMaxBurnManager

        public fun shouldBurnSparkMax(): Boolean = try{
            burnManager.shouldBurn()
        }catch(e: UninitializedPropertyAccessException){
            RobotBase.isReal()
        }


        private val periodicRunnables: MutableList<() -> Unit> = mutableListOf()

        private val simPeriodicRunnables: MutableList<() -> Unit> = mutableListOf()

        public fun addToPeriodicLoop(runnable: () -> Unit){
            periodicRunnables.add(runnable)
        }

        public fun addToSimPeriodicLoop(runnable: () -> Unit){
            simPeriodicRunnables.add(runnable)
        }

        public var LOOP_PERIOD: Time = 0.02.seconds
            private set

        private val noUsbSignalAlert = Alert.warning(text = "No logging to WPILOG is happening; cannot find USB stick")
        private val logReceiverQueueAlert = Alert.error(text = "Logging queue exceeded capacity, data will NOT be logged.")

    }

    private lateinit var robotContainer: ChargerRobotContainer
    private lateinit var autonomousCommand: Command
    private lateinit var testCommand: Command



    // Cancels a command; doing nothing if the command is not yet initialized.
    private inline fun cancelCommand(getCommand: () -> Command){
        try{
            val command = getCommand()
            command.cancel()
        }catch(_: UninitializedPropertyAccessException){}
    }
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        try{
            burnManager = SparkMaxBurnManager(gitData.buildDate)
            LOOP_PERIOD = config.loopPeriod

            val logger = Logger.getInstance()
            setUseTiming(
                RobotBase.isReal() || !config.isReplay
            )

            logger.apply{
                recordMetadata(
                    "Robot", if (RobotBase.isReal()) "REAL" else if (config.isReplay) "REPLAY" else "SIM"
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
                }else if (config.isReplay){
                    // replay mode; sim
                    val path = LogFileUtil.findReplayLog()
                    setReplaySource(WPILOGReader(path))
                    addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")))
                }else{
                    // sim mode
                    logger.addDataReceiver(NTSafePublisher())
                    // maybe add DriverStationSim? idk
                }

                config.extraLoggerConfig(this)

                // no more configuration from this point on
                start()
            }

            LiveWindow.disableAllTelemetry()

            DashboardTuner.tuningMode = config.tuningMode

            PathPlannerServer.startServer(5811)

            // inits robotContainer
            robotContainer = getRobotContainer()


            robotContainer.robotInit()

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
        }catch(e: Exception){
            println("Error has been caught in [robotInit].")
            config.onError(e)
            throw e
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
        try{
            robotContainer.robotPeriodic()
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
        }catch(e: Exception){
            println("Error has been caught in [robotPeriodic].")
            config.onError(e)
            throw e
        }

    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
        try{
            robotContainer.disabledInit()
        }catch(e: Exception){
            println("Error has been caught in [disabledInit].")
            config.onError(e)
            throw e
        }
    }

    override fun disabledPeriodic() {
        try{
            robotContainer.disabledPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [disabledPeriodic].")
            config.onError(e)
            throw e
        }
    }

    /** This autonomous runs the autonomous command selected by your RobotContainer class.  */
    override fun autonomousInit() {
        try{
            cancelCommand{testCommand}
            autonomousCommand = robotContainer.autonomousCommand
            autonomousCommand.schedule()
            robotContainer.autonomousInit()
        }catch(e: Exception){
            println("Error has been caught in [autonomousInit].")
            config.onError(e)
            throw e
        }

    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
        try{
            robotContainer.autonomousPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [autonomousPeriodic].")
            config.onError(e)
            throw e
        }

    }

    override fun teleopInit() {
        try{
            cancelCommand{autonomousCommand}
            cancelCommand{testCommand}
            
            robotContainer.teleopInit()
        }catch(e: Exception){
            println("Error has been caught in [teleopInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {
        try{
            robotContainer.teleopPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [teleopPeriodic].")
            config.onError(e)
            throw e
        }
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        try{
            CommandScheduler.getInstance().cancelAll()
            testCommand = robotContainer.testCommand
            robotContainer.testInit()
            testCommand.schedule()
        }catch(e: Exception){
            println("Error has been caught in [testInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {
        try{
            robotContainer.testPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [testPeriodic].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {
        try{
            robotContainer.simulationInit()
        }catch(e: Exception){
            println("Error has been caught in [simulationInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
        try{
            robotContainer.simulationPeriodic()
            simPeriodicRunnables.forEach{
                it()
            }
        }catch(e: Exception){
            println("Error has been caught in [simulationPeriodic].")
            config.onError(e)
            throw e
        }
    }


}









