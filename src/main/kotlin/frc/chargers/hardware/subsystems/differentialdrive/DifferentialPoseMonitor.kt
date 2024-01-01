package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargerlibexternal.frc6328.PoseEstimator
import frc.chargerlibexternal.frc6995.NomadApriltagUtil
import frc.chargers.hardware.sensors.RobotPoseEstimator
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import org.littletonrobotics.junction.Logger.recordOutput

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
    private val poseSuppliers: List<VisionPoseSupplier> = listOf(),
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseEstimator {


    override val robotPose: UnitPose2d
        get() = poseEstimator.latestPose.ofUnit(meters)

    public constructor(
        vararg poseSuppliers: VisionPoseSupplier,
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
    private val visionUpdates: MutableList<PoseEstimator.TimestampedVisionUpdate> = mutableListOf()

    override fun periodic(){
        val distanceL = leftWheelTravel * wheelTravelPerMotorRadian
        val distanceR = rightWheelTravel * wheelTravelPerMotorRadian

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

        if (poseSuppliers.size > 0){
            visionUpdates.clear()
            poseSuppliers.forEach{
                val measurement = it.robotPoseEstimate
                if (measurement != null){
                    val stdDevVector = NomadApriltagUtil.calculateVisionUncertainty(
                        measurement.value.x.siValue,
                        heading.asRotation2d(),
                        it.cameraYaw.asRotation2d(),
                    )
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
        }


        field.robotPose = poseEstimator.latestPose
        recordOutput("Drivetrain(Differential)/Pose2d",poseEstimator.latestPose)
        recordOutput("Drivetrain(Differential)/realGyroUsedInPoseEstimation", gyro != null)
        recordOutput("Drivetrain(Differential)/realGyroHeadingRad",gyro?.heading?.inUnit(radians) ?: 0.0)
    }
}