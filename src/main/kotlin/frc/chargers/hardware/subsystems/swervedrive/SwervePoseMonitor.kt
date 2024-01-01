package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargerlibexternal.frc6328.PoseEstimator
import frc.chargerlibexternal.frc6328.PoseEstimator.TimestampedVisionUpdate
import frc.chargerlibexternal.frc6995.NomadApriltagUtil
import frc.chargers.advantagekitextensions.recordLatency
import frc.chargers.utils.math.inputModulus
import frc.chargers.hardware.sensors.RobotPoseEstimator
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.chargers.wpilibextensions.kinematics.ModulePositionGroup
import org.littletonrobotics.junction.Logger.*

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
    poseSuppliers: List<VisionPoseSupplier>,
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseEstimator{

    /* Public API */

    public constructor(
        vararg poseSuppliers: VisionPoseSupplier,
        gyro: HeadingProvider? = null,
        startingPose: UnitPose2d = UnitPose2d()
    ): this(
        gyro,
        poseSuppliers.toList(),
        startingPose
    )
    public val field: Field2d = Field2d().also{ SmartDashboard.putData("Field",it) }


    override val robotPose: UnitPose2d get() = poseEstimator.latestPose.ofUnit(meters)

    public val heading: Angle get() = calculatedHeading

    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
        calculatedHeading = pose.rotation
    }

    public fun zeroPose(){
        resetPose(UnitPose2d())
    }

    public fun addPoseSuppliers(vararg suppliers: VisionPoseSupplier){
        this.poseSuppliers.addAll(suppliers)
    }


    /* Private Implementation */


    private val poseEstimator: PoseEstimator = PoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001),
    ).also{
        it.resetPose(startingPose.inUnit(meters))
    }
    private val poseSuppliers = poseSuppliers.toMutableList()
    private var calculatedHeading = Angle(0.0)
    private val previousDistances = Array(4){Distance(0.0)}
    private var lastGyroHeading = Angle(0.0)
    private var wheelDeltas = ModulePositionGroup()
    private val visionUpdates: MutableList<TimestampedVisionUpdate> = mutableListOf()


    override fun periodic(){
        recordLatency("SwervePoseMonitorLoopTime"){
            /*
            Calculates the pose and heading from the data from the swerve modules; results in a Twist2d object.
             */
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
            calculatedHeading += twist.dtheta.ofUnit(radians)
            calculatedHeading = calculatedHeading.inputModulus(0.0.degrees..360.degrees)


            /*
            If a gyro is given, replace the calculated heading with the gyro's heading before adding the twist to the pose estimator.
             */
            if (gyro != null){
                val currentGyroHeading = gyro.heading
                twist.dtheta = (currentGyroHeading - lastGyroHeading).inUnit(radians)
                lastGyroHeading = currentGyroHeading
            }
            poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds),twist)


            /*
            Sends all pose data to the pose estimator.
             */
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
                            TimestampedVisionUpdate(
                                measurement.timestamp.inUnit(seconds),
                                measurement.value.inUnit(meters),
                                stdDevVector
                            )
                        )
                    }
                }
                if (visionUpdates.size != 0) poseEstimator.addVisionData(visionUpdates)
            }



            /*
            Records the robot's pose on the field and in AdvantageScope.
             */
            field.robotPose = poseEstimator.latestPose
            recordOutput("Drivetrain(Swerve)/Pose2d",poseEstimator.latestPose)
            recordOutput("Drivetrain(Swerve)/calculatedHeadingRad",calculatedHeading.inUnit(radians))
            recordOutput("Drivetrain(Swerve)/realGyroUsedInPoseEstimation", gyro != null)
            recordOutput("Drivetrain(Swerve)/realGyroHeadingRad",gyro?.heading?.inUnit(radians) ?: 0.0)
        }
    }





}
