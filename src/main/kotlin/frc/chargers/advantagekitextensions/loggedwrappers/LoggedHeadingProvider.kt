package frc.chargers.advantagekitextensions.loggedwrappers

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.kinematics.swerve.ModulePositionGroup
import org.littletonrobotics.junction.Logger

/**
 * A wrapper over a [HeadingProvider],
 * with AdvantageKit logging, simulation and replay support.
 * This class is intended to be used in other subsystems.
 *
 * Note: the [updateAndProcessInputs] function must be called repeatedly in a periodic loop.
 */
public open class LoggedHeadingProvider(
    private val headingProvider: HeadingProvider = blankHeadingProvider
): HeadingProvider {
    protected val inputs: Inputs = Inputs()
    protected inner class Inputs: ChargerLoggableInputs(){
        public var headingValue: Angle
            by loggedQuantity(Angle(0.0),"headingDegrees", degrees)
    }

    public open fun updateAndProcessInputs(logName: String){
        inputs.headingValue = headingProvider.heading
        // processInputs calls fromLog(which updates the headingValue variable itself)
        // in replay mode,
        // in addition to pushing the data to log.
        Logger.getInstance().processInputs(logName, inputs)
    }

    override val heading: Angle
        get() = inputs.headingValue


    public companion object{
        /**
         * Creates an instance of a [LoggedHeadingProvider]
         *
         * that derives it's heading from wheel positions and the kinematics object of a drivetrain.
         */
        public fun fromSwerveKinematics(
            kinematics: SwerveDriveKinematics,
            getModulePositions: () -> ModulePositionGroup
        ): LoggedHeadingProvider =
            object: LoggedHeadingProvider(){
                
                private val previousDistances = Array(4){Distance(0.0)}
                private var headingValue = Angle(0.0)

                override fun updateAndProcessInputs(logName: String) {
                    val wheelDeltas = ModulePositionGroup()
                    val currentMPs = getModulePositions()


                    wheelDeltas.apply{
                        topLeftDistance = currentMPs.topLeftDistance - previousDistances[0]
                        previousDistances[0] = currentMPs.topLeftDistance

                        topRightDistance = currentMPs.topRightDistance - previousDistances[1]
                        previousDistances[1] = currentMPs.topRightDistance

                        bottomLeftDistance = currentMPs.bottomLeftDistance - previousDistances[2]
                        previousDistances[2] = currentMPs.bottomLeftDistance

                        bottomRightDistance = currentMPs.bottomRightDistance - previousDistances[3]
                        previousDistances[3] = currentMPs.bottomRightDistance

                        topLeftAngle = currentMPs.topLeftAngle
                        topRightAngle = currentMPs.topRightAngle
                        bottomLeftAngle = currentMPs.bottomLeftAngle
                        bottomRightAngle = currentMPs.bottomRightAngle
                    }
                    
                    
                    

                    val twist = kinematics.toTwist2d(*wheelDeltas.toArray())

                    Logger.getInstance().recordOutput("Output twist theta",twist.dtheta)
                    headingValue += twist.dtheta.ofUnit(radians)
                    headingValue %= 360.degrees

                    inputs.headingValue = headingValue

                    Logger.getInstance().processInputs(logName,inputs)
                }
            }
    }




}

private val blankHeadingProvider by lazy{
    object: HeadingProvider{
        override val heading: Angle = Angle(0.0)
    }
}