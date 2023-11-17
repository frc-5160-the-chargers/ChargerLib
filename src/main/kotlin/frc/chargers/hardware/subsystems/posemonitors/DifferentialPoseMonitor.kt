package frc.chargers.hardware.subsystems.posemonitors

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.utils.Measurement
import frc.chargers.external.utils.PoseEstimator
import frc.chargers.wpilibextensions.StandardDeviation
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import org.littletonrobotics.junction.Logger

/**
 * A Helper class used to get the pose of an [EncoderDifferentialDrivetrain].
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
context(EncoderDifferentialDrivetrain)
public class DifferentialPoseMonitor(
    private val gyro: HeadingProvider? = null,
    private val poseSuppliers: List<RobotPoseSupplier> = listOf(),
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseSupplier {
    override val poseStandardDeviation: StandardDeviation
        get() = StandardDeviation.Default
    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() = Measurement(
            poseEstimator.latestPose.ofUnit(meters),
            fpgaTimestamp(),
            true
        )

    public constructor(
        vararg poseSuppliers: RobotPoseSupplier,
        gyro: HeadingProvider? = null,
        startingPose: UnitPose2d = UnitPose2d()
    ): this(
        gyro,
        poseSuppliers.toList(),
        startingPose
    )





    private val poseEstimator = PoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001)
    ).also{
        it.resetPose(startingPose.inUnit(meters))
    }

    private val field = Field2d().also{
        SmartDashboard.putData("Field",it)
    }

    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
    }

    private var previousDistanceL = Distance(0.0)
    private var previousDistanceR = Distance(0.0)

    override fun periodic(){
        val distanceL = inputs.leftAngularPosition * wheelTravelPerMotorRadian
        val distanceR = inputs.rightAngularPosition * wheelTravelPerMotorRadian

        val twist = kinematics.toTwist2d(
            (distanceL-previousDistanceL).inUnit(meters),
            (distanceR-previousDistanceR).inUnit(meters)
        )

        previousDistanceL = distanceL
        previousDistanceR = distanceR

        if (gyro != null){
            twist.dtheta = gyro.heading.inUnit(radians)
        }

        poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds),twist)

        val visionUpdates: MutableList<PoseEstimator.TimestampedVisionUpdate> = mutableListOf()
        poseSuppliers.forEach{
            val measurement = it.robotPoseMeasurement

            if (measurement.isValid){
                val stdDevVector = when(val deviation = it.poseStandardDeviation){
                    is StandardDeviation.Of -> deviation.getVector()

                    is StandardDeviation.Default -> VecBuilder.fill(0.9, 0.9, 0.9)
                }

                visionUpdates.add(
                    PoseEstimator.TimestampedVisionUpdate(
                        measurement.timestamp.inUnit(seconds),
                        measurement.value.inUnit(meters),
                        stdDevVector
                    )
                )
            }
        }
        if (visionUpdates.size != 0) poseEstimator.addVisionData(visionUpdates)


        field.robotPose = poseEstimator.latestPose
        Logger.getInstance().recordOutput("Drivetrain(Differential)/Pose2d",poseEstimator.latestPose)


    }
}