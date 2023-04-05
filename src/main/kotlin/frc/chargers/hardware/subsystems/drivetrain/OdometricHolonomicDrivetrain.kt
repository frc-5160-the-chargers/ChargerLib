package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.Timer
import frc.chargers.hardware.motorcontrol.SwerveModule
import frc.chargers.hardware.sensors.Limelight
import frc.chargers.hardware.sensors.NavX

// how to use the botpose provided:
// by default, if no apriltags are detected when the class is initiated, then all of the pose values will return null.
// once the drivetrain detects apriltags, pose estimation will start.
// to know whether or not pose estimation has started, call drivetrain.startingPointDetected
public class OdometricHolonomicDrivetrain(
    private val limelight: Limelight,
    private val localizationPipeline: Int,
    private val topLeft: SwerveModule,
    private val topRight: SwerveModule,
    private val bottomLeft: SwerveModule,
    private val bottomRight: SwerveModule,
    private val gyro: NavX,
    private val gearRatio: Double = DEFAULT_GEAR_RATIO,
    private val wheelDiameter: Length,
    private val trackWidth: Distance,
    private val wheelBase: Distance
): BasicHolonomicDrivetrain(topLeft,topRight,bottomLeft,bottomRight,gyro,gearRatio,wheelDiameter,trackWidth,wheelBase){
    public var startingPointDetected: Boolean = true
    init{
        limelight.pipeline = localizationPipeline
        if(!limelight.hasTarget){
            startingPointDetected = false
            haltLimelightMeasurement()
        }
    }


    // used to halt limelight measurement and start limelight measurement of the drivetrain.
    // this is a must if you need to temporarily used the limelight for something else.
    public fun restartLimelightMeasurement(){
        limelight.pipeline = localizationPipeline
        useLLMeasurement = true
    }
    public fun haltLimelightMeasurement(){
        useLLMeasurement = false
    }

    private var useLLMeasurement: Boolean = true


    private val drivetrainKinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(
            Translation2d(trackWidth.inUnit(meters)/2,wheelBase.inUnit(meters)/2),
            Translation2d(trackWidth.inUnit(meters)/2,-wheelBase.inUnit(meters)/2),
            Translation2d(-trackWidth.inUnit(meters)/2,wheelBase.inUnit(meters)/2),
            Translation2d(-trackWidth.inUnit(meters)/2,-wheelBase.inUnit(meters)/2)
        )

    private fun modulePosition(module: SwerveModule): SwerveModulePosition {
        return SwerveModulePosition((module.driveEncoder.angularPosition * wheelRadius).inUnit(meters),
            Rotation2d(module.turnEncoder.angularPosition.inUnit(radians))
        )
    }


    private var drivetrainPoseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        drivetrainKinematics,
        gyro.gyroscope.rotation2d,
        arrayOf(
            modulePosition(topLeft),
            modulePosition(topRight),
            modulePosition(bottomLeft),
            modulePosition(bottomRight)
        ),
        Pose2d()
    )


    // returns the raw pose2d object, for use in other wpilib functions.
    public val pose: Pose2d?
        get() = if(startingPointDetected){drivetrainPoseEstimator.estimatedPosition}else{null}


    // note: pose2d's 2 important parameters here are called x and y.
    // thus, these 2 getters use those when getting the x coordinate and y coordinate, respectively.
    // note 2: these values, along with pose, are nullable.
    // They are set to null if the limelight has not detected any apriltags yet.
    public val xShiftFromOrigin: Distance?
        get() = if(startingPointDetected){drivetrainPoseEstimator.estimatedPosition.x.meters}else{null}
    public val yShiftFromOrigin: Distance?
        get() = if(startingPointDetected){drivetrainPoseEstimator.estimatedPosition.y.meters}else{null}

    override fun periodic(){
        if (!startingPointDetected && limelight.hasTarget){
            drivetrainPoseEstimator.resetPosition(
                gyro.gyroscope.rotation2d,
                arrayOf(
                    modulePosition(topLeft),
                    modulePosition(topRight),
                    modulePosition(bottomLeft),
                    modulePosition(bottomRight)),
                Pose2d(limelight.botpose[0],limelight.botpose[1],Rotation2d())
            )
            startingPointDetected = true
            restartLimelightMeasurement()
        }

        // if the swerveDrive function hasn't been called, update the PIDs of the swerve modules.
        if (driveFunctionCalled){
            driveFunctionCalled = false
        }else{
            topLeft.turnPID.calculateOutput()
            topRight.turnPID.calculateOutput()
            bottomLeft.turnPID.calculateOutput()
            bottomRight.turnPID.calculateOutput()
        }

        // updates botpose.
        // each VecBuilder here represents a matrix, which represents "trust" in vision measurements.
        // the higher the numbers, the less "trust" the pose estimator has.
        // this sets the trust very high if megatag is being used(more than 1 tag is present),
        // else, it sets it to the conventional level.
        if (limelight.tv != 0.0 && useLLMeasurement && limelight.pipeline == localizationPipeline){
            // limelight.fiducialResults returns a list with all of the possible targets.
            // megatag is most reliable when more than 1 tag can be seen; thus, this step increases the
            // "trust" in the vision measurement if the limelight has found more than one apriltag.
            if (limelight.fiducialResults.size > 1){
                drivetrainPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.2,0.2,0.2))
            }else{
                drivetrainPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.9,0.9,0.9))
            }

            drivetrainPoseEstimator.addVisionMeasurement(
                Pose2d(limelight.botpose[0],limelight.botpose[1],Rotation2d()),
                Timer.getFPGATimestamp()-(limelight.tl/1000.0)-(limelight.cl/1000.0))
        }


        drivetrainPoseEstimator.update(gyro.gyroscope.rotation2d,
            arrayOf(
                modulePosition(topLeft),
                modulePosition(topRight),
                modulePosition(bottomLeft),
                modulePosition(bottomRight)
            ))

    }


}