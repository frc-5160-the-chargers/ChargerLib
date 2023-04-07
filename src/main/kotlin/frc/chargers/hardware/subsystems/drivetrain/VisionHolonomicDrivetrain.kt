package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import frc.chargers.hardware.motorcontrol.SwerveModule
import frc.chargers.hardware.sensors.Limelight
import frc.chargers.hardware.sensors.NavX
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Transform3d
import frc.chargers.controls.pid.FeedForward
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitFeedForward
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy

/**
 * A convenient way to store vision measurement info.
 */
public data class VisionMeasurementInfo(
    public var trustInMeasurement: Matrix<N3,N1>? = null,
    public var estimatedPose: Pose2d,
    public var timeStamp: Time
)

/**
 * A convenience function to construct a Vision-based [HolonomicDrivetrain] which knows it's position on the field at all times,
 * using
 */
public fun PhotonHolonomicDrivetrain(
    camera: PhotonCamera,
    fieldLayout: AprilTagFieldLayout,
    cameraTransformFromRobot: Transform3d,
    topLeft: SwerveModule,
    topRight: SwerveModule,
    bottomLeft: SwerveModule,
    bottomRight: SwerveModule,
    turnPIDConstants: PIDConstants,
    turnFeedForward: UnitFeedForward<AngleDimension, VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    velocityPIDConstants: PIDConstants,
    velocityFeedForward: UnitFeedForward<VelocityDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    gyro: NavX,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance
): VisionHolonomicDrivetrain = VisionHolonomicDrivetrain(
    topLeft = topLeft,
    topRight = topRight,
    bottomLeft = bottomLeft,
    bottomRight = bottomRight,
    turnPIDConstants = turnPIDConstants,
    turnFeedForward = turnFeedForward,
    velocityPIDConstants = velocityPIDConstants,
    velocityFeedForward = velocityFeedForward,
    gyro = gyro,
    gearRatio = gearRatio,
    wheelDiameter = wheelDiameter,
    trackWidth = trackWidth,
    wheelBase = wheelBase,
    poseUpdator = {previousPose ->
        var poseEstimator = PhotonPoseEstimator(fieldLayout,PoseStrategy.MULTI_TAG_PNP,camera,cameraTransformFromRobot)
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        poseEstimator.setReferencePose(previousPose)
        var result = poseEstimator.update()
        if (result.isPresent){
            var botpose = result.get()
            // return value
            VisionMeasurementInfo(
                trustInMeasurement = VecBuilder.fill(0.9,0.9,0.9),
                estimatedPose = botpose.estimatedPose.toPose2d(),
                timeStamp = botpose.timestampSeconds.seconds
            )
        }else{
            null
        }
    },
    hasTargetBoolSupplier = {camera.latestResult.hasTargets()}
)





/**
 * A convenience function which defines a [VisionHolonomicDrivetrain]
 * using a single limelight as a pose supplier.
 */
public fun LimelightHolonomicDrivetrain(
    limelight: Limelight,
    // these pipelines will be tried, in order, until the limelight can see at least 1 target.
    pipelinesToTry: List<Int>,
    topLeft: SwerveModule,
    topRight: SwerveModule,
    bottomLeft: SwerveModule,
    bottomRight: SwerveModule,
    turnPIDConstants: PIDConstants,
    turnFeedForward: UnitFeedForward<AngleDimension, VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    velocityPIDConstants: PIDConstants,
    velocityFeedForward: UnitFeedForward<VelocityDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
    gyro: NavX,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance): VisionHolonomicDrivetrain = VisionHolonomicDrivetrain(
        topLeft = topLeft,
        topRight = topRight,
        bottomLeft = bottomLeft,
        bottomRight = bottomRight,
        turnPIDConstants = turnPIDConstants,
        turnFeedForward = turnFeedForward,
        velocityPIDConstants = velocityPIDConstants,
        velocityFeedForward = velocityFeedForward,
        gyro = gyro,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        trackWidth = trackWidth,
        wheelBase = wheelBase,
        poseUpdator = {_ ->
            if (!limelight.inUse){
                limelight.pipeline = pipelinesToTry[0]
                var tempCounter = 1
                while(!limelight.hasTarget && tempCounter < pipelinesToTry.size-1){
                    limelight.pipeline = pipelinesToTry[tempCounter]
                    tempCounter += 1
                }



                var poseToReturn: Pose2d = Pose2d(limelight.botpose[0],limelight.botpose[1],Rotation2d())
                var timestamp = (Timer.getFPGATimestamp()-(limelight.tl/1000.0)-(limelight.cl/1000.0)).seconds

                // VecBuilder.fill fills a matrix.
                // the lower the numbers inside of the matrix, the more trust the pose estimator will put in the pose.
                // the limelight implementation of botpose is called megatag, which is very reliable when 2 or more tags are in view.
                // thus, the trust in vision measurement goes up if the size of the limelight.fiducialResults list is higher than 1.
                var trustInVisionMeasurement = if (limelight.fiducialResults.size > 1){
                    VecBuilder.fill(0.3,0.3,0.3)
                }else{
                    VecBuilder.fill(0.9,0.9,0.9)
                }

                // this is the object that the lambda returns.
                VisionMeasurementInfo(
                    trustInMeasurement = trustInVisionMeasurement,
                    estimatedPose = poseToReturn,
                    timeStamp = timestamp
                )
            }else{null}
        },
        hasTargetBoolSupplier = {limelight.hasTarget},
        runWhenPoseEstimationStarts = {limelight.pipeline = pipelinesToTry[0]})

/**
 * A convenience function that lets you define the turn and velocity [PIDConstants] and [FeedForward]
 * within the class definition.
 */
public fun VisionHolonomicDrivetrain(topLeft: SwerveModule,
                                     topRight: SwerveModule,
                                     bottomLeft: SwerveModule,
                                     bottomRight: SwerveModule,
                                     turnPIDConstants: PIDConstants,
                                     turnFeedForward: UnitFeedForward<AngleDimension, VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
                                     velocityPIDConstants: PIDConstants,
                                     velocityFeedForward: UnitFeedForward<VelocityDimension,VoltageDimension> = UnitFeedForward{ _, _ -> 0.0.volts},
                                     gyro: NavX,
                                     gearRatio: Double = DEFAULT_GEAR_RATIO,
                                     wheelDiameter: Length,
                                     trackWidth: Distance,
                                     wheelBase: Distance,
                                     poseUpdator: (Pose2d) -> VisionMeasurementInfo?,
                                     hasTargetBoolSupplier: () -> Boolean,
                                     runWhenPoseEstimationStarts: () -> Unit = {}): VisionHolonomicDrivetrain =
    VisionHolonomicDrivetrain(
        topLeft = topLeft.apply{
            this.turnPIDConstants = turnPIDConstants
            this.turnFeedForward = turnFeedForward
            this.velocityPIDConstants = velocityPIDConstants
            this.velocityFeedForward = velocityFeedForward
        },
        topRight = topRight.apply{
            this.turnPIDConstants = turnPIDConstants
            this.turnFeedForward = turnFeedForward
            this.velocityPIDConstants = velocityPIDConstants
            this.velocityFeedForward = velocityFeedForward
        },
        bottomLeft = bottomLeft.apply{
            this.turnPIDConstants = turnPIDConstants
            this.turnFeedForward = turnFeedForward
            this.velocityPIDConstants = velocityPIDConstants
            this.velocityFeedForward = velocityFeedForward
        },
        bottomRight = bottomRight.apply{
            this.turnPIDConstants = turnPIDConstants
            this.turnFeedForward = turnFeedForward
            this.velocityPIDConstants = velocityPIDConstants
            this.velocityFeedForward = velocityFeedForward
        },
        gyro = gyro,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        trackWidth = trackWidth,
        wheelBase = wheelBase,
        poseUpdator = poseUpdator,
        hasTargetBoolSupplier = hasTargetBoolSupplier,
        runWhenPoseEstimationStarts = runWhenPoseEstimationStarts
    )


/**
 * A [HolonomicDrivetrain] that uses vision measurements to always know where it is on the field.
 */
public open class VisionHolonomicDrivetrain(
    // defines the swerve modules and gyro.
    private val topLeft: SwerveModule,
    private val topRight: SwerveModule,
    private val bottomLeft: SwerveModule,
    private val bottomRight: SwerveModule,
    private val gyro: NavX,
    // defines gearRatio, wheelDiameter, trackWidth and wheelBase.
    private val gearRatio: Double = DEFAULT_GEAR_RATIO,
    private val wheelDiameter: Length,
    private val trackWidth: Distance,
    private val wheelBase: Distance,
    // this lambda serves to add vision measurements to the pose estimator.
    // it returns a VisionMeasurementInfo object, which provides information about
    // the trust of the pose(as a matrix), the pose itself, as well as the timestamp.
    public val poseUpdator: (Pose2d) -> VisionMeasurementInfo?,
    // supplies the drivetrain with a boolean, which should return true if a target is detected.
    public val hasTargetBoolSupplier: () -> Boolean,
    // run once the drivetrain has found it's first target, a.k.a when the drivetrain has started recording pose.
    // optional.
    public val runWhenPoseEstimationStarts: () -> Unit = {}
): BasicHolonomicDrivetrain(topLeft,topRight,bottomLeft,bottomRight,gyro,gearRatio,wheelDiameter,trackWidth,wheelBase){
    private var poseEstimationStarted = false
    private var haltVisionMeasurement = false

    init{
        topLeft.apply{
            driveWheelTravelPerMotorRadian = wheelTravelPerMotorRadian
        }
        topRight.apply{
            driveWheelTravelPerMotorRadian = wheelTravelPerMotorRadian
        }
        bottomLeft.apply{
            driveWheelTravelPerMotorRadian = wheelTravelPerMotorRadian
        }
        bottomRight.apply{
            driveWheelTravelPerMotorRadian = wheelTravelPerMotorRadian
        }

        // if no target is detected when the drivetrain is initialized, then all of the pose variables are invalid.
        poseEstimationStarted = hasTargetBoolSupplier()

    }

    public fun addVisionMeasurement(info: VisionMeasurementInfo){
        drivetrainPoseEstimator.setVisionMeasurementStdDevs(info.trustInMeasurement)
        drivetrainPoseEstimator.addVisionMeasurement(info.estimatedPose,info.timeStamp.inUnit(seconds))
    }




    // used to halt limelight measurement and start limelight measurement of the drivetrain.
    // this is a must if you need to temporarily used the limelight for something else.
    public fun restartVisionMeasurement(){
        haltVisionMeasurement = false
        runWhenPoseEstimationStarts()
    }
    public fun stopVisionMeasurement(){
        haltVisionMeasurement = true
    }


    override fun periodic(){
        super.periodic()
        var updateInfo: VisionMeasurementInfo? = if(pose == null){null}else{poseUpdator(pose!!)}
        if (updateInfo != null) {
            if (!poseEstimationStarted && hasTargetBoolSupplier()) {
                drivetrainPoseEstimator.resetPosition(
                    gyro.gyroscope.rotation2d,
                    arrayOf(
                        modulePosition(topLeft),
                        modulePosition(topRight),
                        modulePosition(bottomLeft),
                        modulePosition(bottomRight)
                    ),
                    updateInfo.estimatedPose
                )
                poseEstimationStarted = true
                runWhenPoseEstimationStarts()
            }


            if (hasTargetBoolSupplier() && !haltVisionMeasurement && poseEstimationStarted) {
                if(updateInfo.trustInMeasurement != null){
                    drivetrainPoseEstimator.setVisionMeasurementStdDevs(updateInfo.trustInMeasurement)
                }
                drivetrainPoseEstimator.addVisionMeasurement(
                    updateInfo.estimatedPose,
                    updateInfo.timeStamp.inUnit(seconds)
                )
            }
        }




    }


}