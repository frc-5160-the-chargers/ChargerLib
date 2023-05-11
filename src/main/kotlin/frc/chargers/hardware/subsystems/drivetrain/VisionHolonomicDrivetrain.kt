package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Rotation2d
import frc.chargers.hardware.motorcontrol.SwerveModule
import frc.chargers.hardware.sensors.cameras.Limelight
import frc.chargers.hardware.sensors.NavX
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import frc.chargers.controls.pid.FeedForward
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitFeedForward

import org.photonvision.PhotonCamera


// note: These imports are where the SwerveDrivePoseEstimator.function() calls are coming from!!!!
import frc.chargers.utils.setInitialPoseWithLimelights
import frc.chargers.utils.setInitialPoseWithPhotonVision
import frc.chargers.utils.updateWithLimelights
import frc.chargers.utils.updateWithPhotonVision

/**
 * A convenience function to construct a [VisionHolonomicDrivetrain]
 * using an unlimited amount of limelights and photoncameras as pose suppliers.
 */
public fun HybridVisionHolonomicDrivetrain(
    limelights: List<Limelight>,
    limelightPipelines: List<Int>,
    photonCameras: List<PhotonCamera>,
    photonCameraTransforms: List<Transform3d>,
    fieldLayout: AprilTagFieldLayout,
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
    topLeft,
    topRight,
    bottomLeft,
    bottomRight,
    gyro,
    gearRatio,
    wheelDiameter,
    trackWidth,
    wheelBase,
    poseUpdater = {poseEstimator ->
        poseEstimator.updateWithLimelights(limelights,limelightPipelines)
        poseEstimator.updateWithPhotonVision(photonCameras,photonCameraTransforms,fieldLayout)
    },
    poseSetter = {gyroRotation,modulePosArray,poseEstimator ->
        var poseHasSetWithLimelights = poseEstimator.setInitialPoseWithLimelights(limelights,limelightPipelines,gyroRotation,modulePosArray)
        if(!poseHasSetWithLimelights){
            // returns true/false btw.
            poseEstimator.setInitialPoseWithPhotonVision(photonCameras,
                photonCameraTransforms,fieldLayout,gyroRotation,modulePosArray)
        }else{
            true
        }
    }

)

/**
 * A convenience function to construct a Vision-based [HolonomicDrivetrain] which knows it's position on the field at all times,
 * using an unlimited amount of [PhotonCamera]s.
 */
public fun PhotonHolonomicDrivetrain(
    cameras: List<PhotonCamera>,
    fieldLayout: AprilTagFieldLayout,
    cameraTransformsFromRobotCenter: List<Transform3d>,
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
    poseUpdater = {poseEstimator -> poseEstimator.updateWithPhotonVision(cameras,cameraTransformsFromRobotCenter,fieldLayout)},
    poseSetter = {gyroRotation, modulePosArray, poseEstimator ->
        poseEstimator.setInitialPoseWithPhotonVision(cameras,cameraTransformsFromRobotCenter,fieldLayout,gyroRotation,modulePosArray)
    }
)







/**
 * A convenience function which defines a [VisionHolonomicDrivetrain]
 * using an unlimited amount of [Limelight]s as pose suppliers.
 */
public fun LimelightHolonomicDrivetrain(
    cameras: List<Limelight>,
    // these pipelines will be tried, in order, until the limelight can see at least 1 target.
    // note: each limelight should be using the same pipelines.
    // for example, if you have limelight 1 and limelight 2, both of them should be using pipeline 7(for example)
    // to detect
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
        poseUpdater = {poseEstimator -> poseEstimator.updateWithLimelights(cameras,pipelinesToTry)},
        poseSetter = {gyroRotation, modulePosArray, poseEstimator ->
            poseEstimator.setInitialPoseWithLimelights(cameras,pipelinesToTry,gyroRotation,modulePosArray)})

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
                                     poseUpdater: (SwerveDrivePoseEstimator) -> Unit,
                                     poseSetter: (Rotation2d,Array<SwerveModulePosition>,SwerveDrivePoseEstimator) -> Boolean = {_,_,_ -> true}): VisionHolonomicDrivetrain =
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
        poseUpdater = poseUpdater,
        poseSetter = poseSetter
    )



/**
 * A [HolonomicDrivetrain] that fuses vision measurements with odometry
 * to always know where it is on the field.
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
    // it takes in a swerveDrivePoseEstimator, then updates it with all of the vision measurement info needed.
    public val poseUpdater: (SwerveDrivePoseEstimator) -> Unit,
    // this serves to "set" the pose.
    // when the drivetrain is initialized, sometimes, the vision sensors cannot see any possible vision targets to set a pose.
    // the poseSetter will be run(in the periodic loop of the subsystem) until it returns true(aka when the pose has been set.)
    public val poseSetter: (Rotation2d,Array<SwerveModulePosition>,SwerveDrivePoseEstimator) -> Boolean = {_,_,_ -> true},
): EncoderHolonomicDrivetrain(topLeft,topRight,bottomLeft,bottomRight,gyro,gearRatio,wheelDiameter,trackWidth,wheelBase){
    private var poseEstimationStarted = false
    private var haltVisionMeasurement = false

    private val currentModulePositions: Array<SwerveModulePosition>
        get() = arrayOf(
            modulePosition(topLeft),
            modulePosition(topRight),
            modulePosition(bottomLeft),
            modulePosition(bottomRight)
        )

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
        poseEstimationStarted = poseSetter(gyro.gyroscope.rotation2d,arrayOf(
            modulePosition(topLeft),
            modulePosition(topRight),
            modulePosition(bottomLeft),
            modulePosition(bottomRight)
        ),drivetrainPoseEstimator)

    }



    // used to halt limelight measurement and start limelight measurement of the drivetrain.
    // this is a must if you need to temporarily used the limelight for something else.
    public fun restartVisionMeasurement(){
        haltVisionMeasurement = false
        poseSetter(gyro.gyroscope.rotation2d,arrayOf(
            modulePosition(topLeft),
            modulePosition(topRight),
            modulePosition(bottomLeft),
            modulePosition(bottomRight)
        ),drivetrainPoseEstimator)
    }
    public fun stopVisionMeasurement(){
        haltVisionMeasurement = true
    }


    override fun periodic(){
        super.periodic()
        if (!poseEstimationStarted){
            poseEstimationStarted = poseSetter(gyro.gyroscope.rotation2d,arrayOf(
                modulePosition(topLeft),
                modulePosition(topRight),
                modulePosition(bottomLeft),
                modulePosition(bottomRight)
            ),drivetrainPoseEstimator)
        }
        if (poseEstimationStarted && !haltVisionMeasurement){
            poseUpdater(drivetrainPoseEstimator)
        }

    }


}