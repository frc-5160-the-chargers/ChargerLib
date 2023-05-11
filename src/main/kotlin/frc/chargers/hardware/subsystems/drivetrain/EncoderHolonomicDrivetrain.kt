package frc.chargers.hardware.subsystems.drivetrain


import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.SwerveModule
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.cameras.VisionCamera3d
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import frc.chargers.hardware.sensors.encoders.Encoder
import kotlin.math.sin
import kotlin.math.cos
import kotlin.math.sqrt



// public fun sparkMaxDrivetrain: EncoderHolonomicDrivetrain
// accepts configuration for drive and turn motors
// public fun talonFXDrivetrain: EncoderHolonomicDrivetrain
// accepts configuration for drive and turn motors
// public fun MultiMotorDrivetrain<turnMotor: EncoderMotorController,driveMotor: EncoderMotorController>


public open class EncoderHolonomicDrivetrain(
    private val visionCameras: List<VisionCamera3d>
)



// note: default gear ratio defined in EncoderDifferentialDriveTrain.
// They're in the same package and thus no import
// note: trackwidth is horizontal and wheelBase is vertical
// second note: DEFAULT_GEAR_RATIO is defined in encoderdifferentialdrivetrain.
/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 * This also provides basic odometry(in the form of SwerveDrivePoseEstimator); however, it's less reliable than
 * [VisionHolonomicDrivetrain], which should always be used if a vision sensor(limelight or photon camera）is available.
 */
public open class TesttestTest(
    private val topLeft: SwerveModule,
    private val topRight: SwerveModule,
    private val bottomLeft: SwerveModule,
    private val bottomRight: SwerveModule,
    private val gyro: NavX,
    private val gearRatio: Double = DEFAULT_GEAR_RATIO,
    private val wheelDiameter: Length,
    private val trackWidth: Distance,
    private val wheelBase: Distance,
    private val startingPose: Pose2d = Pose2d()
): SubsystemBase(), HolonomicDrivetrain{
    // do drivetrain.fieldRelativeDrive = false to turn this option off.
    public var fieldRelativeDrive: Boolean = true
    
    init {
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
    }



    public val overallEncoder: Encoder = AverageEncoder(
        topLeft.driveEncoder,
        topRight.driveEncoder,
        bottomLeft.driveEncoder,
        bottomRight.driveEncoder)


    // all borrowed from EncoderDifferentialDriveTrain. see more information there.
    // note: an EncoderHolonomicDriveTrain CANNOT provide heading, and thus is NOT a headingProvider.
    // it relies on the navX reading in order to be field-centric
    private val diagonal: Distance = sqrt((wheelBase.inUnit(meters) * wheelBase.inUnit(meters) + trackWidth.inUnit(meters) * trackWidth.inUnit(meters))).meters

    public val wheelRadius: Distance = wheelDiameter / 2
    public var wheelTravelPerMotorRadian: Distance = gearRatio * wheelRadius

    private val distanceOffset: Distance = overallEncoder.angularPosition * wheelTravelPerMotorRadian
    public val distanceTraveled: Distance
        get() =
            (overallEncoder.angularPosition *
                    wheelTravelPerMotorRadian) - distanceOffset
    public val velocity: Velocity
        get() =
            overallEncoder.angularVelocity *
                    wheelTravelPerMotorRadian

    public var driveFunctionCalled: Boolean = false
    public var setDesiredStatesFunctionCalled: Boolean = false

    private val drivetrainKinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(
            Translation2d(trackWidth.inUnit(meters)/2,wheelBase.inUnit(meters)/2),
            Translation2d(trackWidth.inUnit(meters)/2,-wheelBase.inUnit(meters)/2),
            Translation2d(-trackWidth.inUnit(meters)/2,wheelBase.inUnit(meters)/2),
            Translation2d(-trackWidth.inUnit(meters)/2,-wheelBase.inUnit(meters)/2)
        )

    // convenience function to save the amount of stuff I have to type.
    public fun modulePosition(module: SwerveModule): SwerveModulePosition {
        return SwerveModulePosition((module.driveEncoder.angularPosition * wheelRadius).inUnit(meters),
            Rotation2d(module.turnEncoder.angularPosition.inUnit(radians))
        )
    }

    public var drivetrainPoseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        drivetrainKinematics,
        gyro.gyroscope.rotation2d,
        arrayOf(
            modulePosition(topLeft),
            modulePosition(topRight),
            modulePosition(bottomLeft),
            modulePosition(bottomRight)
        ),
        startingPose
    )

    // returns the raw pose2d object, for use in other wpilib functions.
    public open val pose: Pose2d?
        get() = drivetrainPoseEstimator.estimatedPosition


    // note: pose2d's 2 important parameters here are called x and y.
    // thus, these 2 getters use those when getting the x coordinate and y coordinate, respectively.
    // note 2: these values, along with pose, are nullable.
    // They are set to null if the limelight has not detected any apriltags yet.
    public open val xDistanceFromOrigin: Distance?
        get() = drivetrainPoseEstimator.estimatedPosition.x.meters
    public open val yDistanceFromOrigin: Distance?
        get() = drivetrainPoseEstimator.estimatedPosition.y.meters


    // sets desired moduleStates for each of the swerve modules.
    public fun setDesiredStates(vararg moduleStates: SwerveModuleState){
        topLeft.setDesiredModuleState(moduleStates[0])
        topRight.setDesiredModuleState(moduleStates[1])
        bottomLeft.setDesiredModuleState(moduleStates[2])
        bottomRight.setDesiredModuleState(moduleStates[3])
        setDesiredStatesFunctionCalled = true
    }


    // counter-intuitively, xPower is straight, and yPower is side-to-side.
    override fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double){
        // a gyro input into the swerveDrive allows it to remain field-centric; optional


        val forwardPower: Double = if(!fieldRelativeDrive){xPower}else{
            xPower*cos(gyro.gyroscope.heading.inUnit(degrees)) + yPower * sin(gyro.gyroscope.heading.inUnit(degrees))
        }
        val sidePower: Double = if(!fieldRelativeDrive){yPower}else{
            -xPower * sin(gyro.gyroscope.heading.inUnit(degrees)) + yPower * cos(gyro.gyroscope.heading.inUnit(degrees))
        }


        var A: Double = sidePower - rotationPower * (wheelBase.inUnit(meters)/diagonal.inUnit(meters))
        var B: Double = sidePower + rotationPower * (wheelBase.inUnit(meters)/diagonal.inUnit(meters))
        var C: Double = forwardPower - rotationPower * (trackWidth.inUnit(meters)/diagonal.inUnit(meters))
        var D: Double = forwardPower + rotationPower * (trackWidth.inUnit(meters)/diagonal.inUnit(meters))

        var topRightPower: Double = sqrt(B*B + C*C)
        var topLeftPower: Double = sqrt(B*B+D*D)
        var bottomLeftPower: Double = sqrt(A*A+D*D)
        var bottomRightPower: Double = sqrt(A*A+C*C)


        // the following "normalizes" the wheel
        var max: Double? = listOf(topRightPower,topLeftPower,bottomLeftPower,bottomRightPower).maxOrNull()


        if (max != null && max > 1.0){
            topRightPower /= max
            topLeftPower /= max
            bottomRightPower /= max
            bottomLeftPower /= max
        }



        var topRightAngle: Angle = atan(B/C)
        var topLeftAngle: Angle = atan(B/D)
        var bottomRightAngle: Angle = atan(A/D)
        var bottomLeftAngle: Angle = atan(A/C)


        topLeft.setDirectionalPower(topLeftAngle,topLeftPower)
        topRight.setDirectionalPower(topRightAngle,topRightPower)
        bottomLeft.setDirectionalPower(bottomLeftAngle,bottomLeftPower)
        bottomRight.setDirectionalPower(bottomRightAngle,bottomRightPower)

        driveFunctionCalled = true

    }

    /*
    Drives the drivetrain with a specific power at a specified angle.
     */
    override fun directionalDrive(xPower: Double, angle: Angle){
        topLeft.setDirectionalPower(angle,xPower)
        topRight.setDirectionalPower(angle,xPower)
        bottomLeft.setDirectionalPower(angle,xPower)
        bottomRight.setDirectionalPower(angle,xPower)
    }

    /*
    Stops the drivetrain.
     */
    override fun stop(){
        topLeft.halt()
        topRight.halt()
        bottomLeft.halt()
        bottomRight.halt()
    }

    /*
    Makes the drivetrain rotate in place, with a specific power.
     */
    override fun rotateInPlace(power: Double) {
        topLeft.setDirectionalPower(45.degrees,power)
        topRight.setDirectionalPower(-(45.degrees),power)
        bottomLeft.setDirectionalPower(45.degrees,power)
        bottomRight.setDirectionalPower(-(45.degrees),power)
    }

    /*
    Periodically updates the PIDs so that they won't break.
    Note: both setDirectionalOutput and topLeft.turnPID.calculateOutput() call the calculateOutput()
    function on the PID.
     */
    override fun periodic(){
        // outputs aren't used; these are just to make sure that
        // the PIDs are calculated repeatedly as to avoid breaking the PID controllers.
        if (driveFunctionCalled){
            driveFunctionCalled = false
        }else{
            topLeft.turnPID.calculateOutput()
            topRight.turnPID.calculateOutput()
            bottomLeft.turnPID.calculateOutput()
            bottomRight.turnPID.calculateOutput()
        }

        if(setDesiredStatesFunctionCalled){
            setDesiredStatesFunctionCalled = false
        }else{
            topLeft.velocityPID.calculateOutput()
            topRight.velocityPID.calculateOutput()
            bottomLeft.velocityPID.calculateOutput()
            bottomRight.velocityPID.calculateOutput()
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