package frc.chargers.hardware.subsystems.posemonitors

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.utils.Measurement
import frc.chargerlibexternal.utils.PoseEstimator
import frc.chargerlibexternal.utils.PoseEstimator.TimestampedVisionUpdate
import frc.chargers.utils.NullableMeasurement
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.kinematics.swerve.ModulePositionGroup
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * A Helper class used to get the pose of an [EncoderHolonomicDrivetrain],
 * with heading-supplying utilities.
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
context(EncoderHolonomicDrivetrain)
public class SwervePoseMonitor(
    private val gyro: HeadingProvider? = null,
    private val poseSuppliers: List<RobotPoseSupplier>,
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseSupplier{


    /* Public API */

    public constructor(
        vararg poseSuppliers: RobotPoseSupplier,
        gyro: HeadingProvider? = null,
        startingPose: UnitPose2d = UnitPose2d()
    ): this(
        gyro,
        poseSuppliers.toList(),
        startingPose
    )

    override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default
    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() = Measurement(
            poseEstimator.latestPose.ofUnit(meters),
            fpgaTimestamp()
        )
    override val robotPose: UnitPose2d get() = robotPoseMeasurement.value
    public val heading: Angle
        get() = headingInputs.calculatedHeading
    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
        headingInputs.calculatedHeading = pose.rotation
    }








    /* Private functions/data */


    /**
     * These are the Inputs classes of the pose monitor,
     * which
     */
    private val headingInputs = HeadingInputs()
    private val poseSupplierInputs = PoseSupplierInputs()
    private val previousDistances = Array(4){Distance(0.0)}
    private val field: Field2d = Field2d().also{
        SmartDashboard.putData("Field",it)
    }
    private val poseEstimator: PoseEstimator = PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.00001),).also{
        it.resetPose(startingPose.inUnit(meters))
    }
    private fun updateInputs(calculatedHeading: Angle){
        headingInputs.apply{
            gyroUsed = gyro != null
            gyroInputHeading = (gyro?.heading ?: Angle(0.0)).inputModulus(0.0.degrees..360.degrees)
            this.calculatedHeading = calculatedHeading
        }

        poseSupplierInputs.apply{
            poseMeasurements = poseSuppliers.map{it.robotPoseMeasurement}
        }
    }







    override fun periodic(){
        /*
        Calculates the pose and heading from the data from the swerve modules; results in a Twist2d object.
         */
        val wheelDeltas = ModulePositionGroup()
        val currentMPs = currentModulePositions
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
        val currentEstimatedHeading =
            (headingInputs.calculatedHeading + twist.dtheta.ofUnit(radians)).inputModulus(0.0.degrees..360.degrees)


        /*
        Updates all data, pushes it to log, and overrides it with previously-logged data if replay mode is active.
         */
        updateInputs(calculatedHeading = currentEstimatedHeading)
        Logger.getInstance().processInputs("Drivetrain(Swerve)/HeadingData",headingInputs)
        Logger.getInstance().processInputs("Drivetrain(Swerve)/PoseData",poseSupplierInputs)


        /*
        If a gyro is given, replace the calculated heading with the gyro's heading before adding the twist to the pose estimator.
         */
        if (gyro != null){
            twist.dtheta = headingInputs.gyroInputHeading.inUnit(radians)
        }
        poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds),twist)


        /*
        Sends all pose data to the pose estimator.
         */
        val visionUpdates: MutableList<TimestampedVisionUpdate> = mutableListOf()
        for (i in 0..<poseSupplierInputs.poseMeasurements.size){
            val measurement = poseSupplierInputs.poseMeasurements[i]

            if (measurement.nullableValue != null){
                val stdDevVector = when(val deviation = poseSupplierInputs.stdDevs[i]){
                    is StandardDeviation.Of -> deviation.getVector()

                    is StandardDeviation.Default -> VecBuilder.fill(0.9, 0.9, 0.9)
                }

                visionUpdates.add(
                    TimestampedVisionUpdate(
                        measurement.timestamp.inUnit(seconds),
                        measurement.nullableValue.inUnit(meters),
                        stdDevVector
                    )
                )
            }
        }
        if (visionUpdates.size != 0) poseEstimator.addVisionData(visionUpdates)


        /*
        Records the robot's pose on the field and in AdvantageScope.
         */
        field.robotPose = poseEstimator.latestPose
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/Pose2d",poseEstimator.latestPose)
    }


    /**
     * Handles data storage for the pose outputs of the pose suppliers.
     */
    private inner class PoseSupplierInputs: LoggableInputs{
        var poseMeasurements = poseSuppliers.map{it.robotPoseMeasurement}
        val stdDevs: List<StandardDeviation> = poseSuppliers.map{it.poseStandardDeviation}

        private fun LogTable.sendPose(key: String, pose: UnitPose2d?){
            put("$key/PoseIsValid", pose != null)
            if (pose != null){
                put("$key/x(meters)", pose.x.inUnit(meters))
                put("$key/y(meters)", pose.y.inUnit(meters))
                put("$key/rotation(rad)", pose.rotation.inUnit(radians))
            }else{
                put("$key/x(meters)", 0.0)
                put("$key/y(meters)", 0.0)
                put("$key/rotation(rad)", 0.0)
            }
        }

        private fun LogTable.getPose(key: String): UnitPose2d?{
            if (getBoolean("$key/PoseIsValid",false)) return null

            return UnitPose2d(
                getDouble("$key/x(meters)", 0.0).ofUnit(meters),
                getDouble("$key/y(meters)", 0.0).ofUnit(meters),
                getDouble("$key/rotation(rad)", 0.0).ofUnit(radians)
            )
        }

        /**
         * Pushes data to the log table.
         */
        override fun toLog(table: LogTable?) {
            table?.apply{
                for (i in 1..poseMeasurements.size){
                    sendPose("Pose Source $i", poseMeasurements[i-1].nullableValue)
                    put("Pose Source $i/timestampSecs", poseMeasurements[i-1].timestamp.inUnit(seconds))
                }
            }
        }

        /**
         * Overrides data from the log data in replay mode.
         */
        override fun fromLog(table: LogTable?) {
            table?.apply{
                val newMeasurements = mutableListOf<NullableMeasurement<UnitPose2d>>()
                for (i in 1..poseMeasurements.size){
                    newMeasurements.add(
                        NullableMeasurement(
                            nullableValue = getPose("Pose Source $i"),
                            timestamp = getDouble("Pose Source $i/timestampSecs",fpgaTimestamp().inUnit(seconds)).ofUnit(seconds)
                        )
                    )
                }
                poseMeasurements = newMeasurements
            }
        }
    }

    /**
     * Handles the gyro data retreived from the pose estimator's heading estimation, as well as other heading data.
     *
     * fromLog and toLog functions are auto-generated using the ChargerLoggableInputs() abstract class.
     */
    private inner class HeadingInputs: ChargerLoggableInputs(){
        var gyroUsed by loggedBoolean("realGyroUsedInEstimation", false)
        var gyroInputHeading by loggedQuantity(degrees, "gyroInputHeading", Angle(0.0))
        var calculatedHeading by loggedQuantity(degrees, "calculatedHeadingDeg", Angle(0.0))
    }




}
