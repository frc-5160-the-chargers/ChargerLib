package frc.chargers.hardware.subsystems.posemonitors

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
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
import frc.chargers.utils.PoseEstimator
import frc.chargers.utils.PoseEstimator.TimestampedVisionUpdate
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.kinematics.swerve.ModulePositionGroup
import org.littletonrobotics.junction.Logger

/**
 * A Helper class used to get the pose of an [EncoderHolonomicDrivetrain],
 * with heading-supplying utilities.
 */

context(EncoderHolonomicDrivetrain)
public class SwervePoseMonitor(
    private val gyro: HeadingProvider? = null,
    private val poseSuppliers: List<RobotPoseSupplier>,
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseSupplier, HeadingProvider{

    public constructor(
        vararg poseSuppliers: RobotPoseSupplier,
        gyro: HeadingProvider? = null,
        startingPose: UnitPose2d = UnitPose2d()
    ): this(
        gyro,
        poseSuppliers.toList(),
        startingPose
    )



    override val poseStandardDeviation: StandardDeviation
        get() = StandardDeviation.Default
    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() = Measurement(
            poseEstimator.latestPose.ofUnit(meters),
            fpgaTimestamp(),
            true
        )
    override val heading: Angle
        get() = headingInputs.calculatedHeading




    private val headingInputs = HeadingInputs()
    private inner class HeadingInputs: ChargerLoggableInputs(){
        var gyroUsed by loggedBoolean("realGyroUsedInEstimation", false)
        var gyroInputHeading by loggedQuantity(degrees, "gyroInputHeading", Angle(0.0))
        var calculatedHeading by loggedQuantity(degrees, "calculatedHeadingDeg", Angle(0.0))
    }

    private fun updateInputs(calculatedHeading: Angle){
        headingInputs.apply{
            gyroUsed = gyro != null
            gyroInputHeading = gyro?.heading ?: Angle(0.0)
            this.calculatedHeading = calculatedHeading
        }
    }




    private val previousDistances = Array(4){Distance(0.0)}

    private val field: Field2d = Field2d().also{
        SmartDashboard.putData("Field",it)
    }



    private val poseEstimator: PoseEstimator = PoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001),
    ).also{
        it.resetPose(startingPose.inUnit(meters))
    }


    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
        headingInputs.calculatedHeading = pose.rotation
    }




    override fun periodic(){
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
            (headingInputs.calculatedHeading + twist.dtheta.ofUnit(radians)) % 360.degrees

        updateInputs(calculatedHeading = currentEstimatedHeading)

        Logger.getInstance().processInputs("Drivetrain(Swerve)/HeadingData",headingInputs)

        if (gyro != null){
            twist.dtheta = headingInputs.gyroInputHeading.inUnit(radians)
        }
        poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds),twist)


        val visionUpdates: MutableList<TimestampedVisionUpdate> = mutableListOf()
        poseSuppliers.forEach{
            val measurement = it.robotPoseMeasurement

            if (measurement.isValid){
                val stdDevVector = when(val deviation = it.poseStandardDeviation){
                    is StandardDeviation.Of -> deviation.getVector()

                    is StandardDeviation.Default -> VecBuilder.fill(0.9, 0.9, 0.9)
                }

                visionUpdates.add(
                    TimestampedVisionUpdate(
                        measurement.timestamp.inUnit(seconds),
                        measurement.value.inUnit(meters),
                        stdDevVector
                    )
                )
            }
        }
        if (visionUpdates.size != 0) poseEstimator.addVisionData(visionUpdates)


        field.robotPose = poseEstimator.latestPose
        Logger.getInstance().recordOutput("Drivetrain(Swerve)/Pose2d",poseEstimator.latestPose)


    }




}
