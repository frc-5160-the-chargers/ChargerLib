package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.swerve.NonConfigurableHolonomicModule
import frc.chargers.hardware.motorcontrol.MotorConfiguration

import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.RobotPoseSupplier
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.Timer
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.geometry.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.asRotation2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import kotlin.math.sqrt
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.hardware.motorcontrol.swerve.HolonomicModule
import frc.chargers.hardware.motorcontrol.swerve.ModuleConfiguration
import frc.chargers.utils.WheelRatioProvider
import frc.chargers.wpilibextensions.kinematics.*
/**
 * A convenience function to create a [EncoderHolonomicDrivetrain]
 * using Talon FX motor controllers.
 *
 * @see EncoderHolonomicDrivetrain
 */
public fun talonFXHolonomicDrivetrain(
    topLeft: HolonomicModule<TalonFXConfiguration, TalonFXConfiguration>,
    topRight: HolonomicModule<TalonFXConfiguration, TalonFXConfiguration>,
    bottomLeft: HolonomicModule<TalonFXConfiguration, TalonFXConfiguration>,
    bottomRight: HolonomicModule<TalonFXConfiguration, TalonFXConfiguration>,
    gyro: HeadingProvider,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    startingPose: UnitPose2d = UnitPose2d(),
    fieldRelativeDrive: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier,
    configure: ModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration>.() -> Unit = {}
): EncoderHolonomicDrivetrain = EncoderHolonomicDrivetrain(
    topLeft, topRight, bottomLeft, bottomRight, gyro, gearRatio, wheelDiameter, trackWidth, wheelBase, startingPose, fieldRelativeDrive,
    *poseSuppliers,
    configuration = ModuleConfiguration(TalonFXConfiguration(),TalonFXConfiguration()).apply(configure),
)

/**
 * A convenience function to create a [EncoderHolonomicDrivetrain]
 * using SparkMax motor controllers.
 *
 * @see EncoderHolonomicDrivetrain
 */
public fun sparkMaxHolonomicDrivetrain(
    topLeft: HolonomicModule<SparkMaxConfiguration, SparkMaxConfiguration>,
    topRight: HolonomicModule<SparkMaxConfiguration, SparkMaxConfiguration>,
    bottomLeft: HolonomicModule<SparkMaxConfiguration, SparkMaxConfiguration>,
    bottomRight: HolonomicModule<SparkMaxConfiguration, SparkMaxConfiguration>,
    gyro: HeadingProvider,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    startingPose: UnitPose2d = UnitPose2d(),
    fieldRelativeDrive: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier,
    configure: ModuleConfiguration<SparkMaxConfiguration, SparkMaxConfiguration>.() -> Unit = {},
): EncoderHolonomicDrivetrain = EncoderHolonomicDrivetrain(
    topLeft, topRight, bottomLeft, bottomRight, gyro, gearRatio, wheelDiameter, trackWidth, wheelBase, startingPose, fieldRelativeDrive,
    *poseSuppliers,
    configuration = ModuleConfiguration(SparkMaxConfiguration(),SparkMaxConfiguration()).apply(configure),
)

/**
 * A convenience function to create an [EncoderHolonomicDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <TMC: MotorConfiguration, DMC: MotorConfiguration> EncoderHolonomicDrivetrain(
    topLeft: HolonomicModule<TMC, DMC>,
    topRight: HolonomicModule<TMC, DMC>,
    bottomLeft: HolonomicModule<TMC, DMC>,
    bottomRight: HolonomicModule<TMC, DMC>,
    gyro: HeadingProvider,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    trackWidth: Distance,
    wheelBase: Distance,
    startingPose: UnitPose2d = UnitPose2d(),
    fieldRelativeDrive: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier,
    configuration: ModuleConfiguration<TMC, DMC>? = null,
): EncoderHolonomicDrivetrain = EncoderHolonomicDrivetrain(
    topLeft.apply{
        if(configuration != null){
            configure(configuration)
        }
    },
    topRight.apply{
        if(configuration != null){
            configure(configuration)
        }
    },
    bottomLeft.apply{
        if(configuration != null){
            configure(configuration)
        }
    },
    bottomRight.apply{
        if(configuration != null){
            configure(configuration)
        }
    }, gyro, gearRatio, wheelDiameter, trackWidth, wheelBase, startingPose, fieldRelativeDrive, *poseSuppliers
)




// note: trackwidth is horizontal and wheelBase is vertical
// second note: DEFAULT_GEAR_RATIO is defined in encoderdifferentialdrivetrain.
/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 */
public class EncoderHolonomicDrivetrain(
    private val topLeft: NonConfigurableHolonomicModule,
    private val topRight: NonConfigurableHolonomicModule,
    private val bottomLeft: NonConfigurableHolonomicModule,
    private val bottomRight: NonConfigurableHolonomicModule,
    public val gyro: HeadingProvider,
    override val gearRatio: Double = DEFAULT_GEAR_RATIO,
    override val wheelDiameter: Length,
    private val trackWidth: Distance,
    private val wheelBase: Distance,
    public val startingPose: UnitPose2d = UnitPose2d(),
    // do drivetrain.fieldRelativeDrive = false to turn this option off.
    public var fieldRelativeDrive: Boolean = true,
    vararg poseSuppliers: RobotPoseSupplier
): SubsystemBase(), RobotPoseSupplier, WheelRatioProvider{

    private val allPoseSuppliers: MutableList<RobotPoseSupplier> = poseSuppliers.toMutableList()


    private val overallEncoder: Encoder = AverageEncoder(
        topLeft.distanceMeasurementEncoder,
        topRight.distanceMeasurementEncoder,
        bottomLeft.distanceMeasurementEncoder,
        bottomRight.distanceMeasurementEncoder)


    // all borrowed from EncoderDifferentialDriveTrain. see more information there.
    // note: an EncoderHolonomicDriveTrain CANNOT provide heading, and thus is NOT a headingProvider.
    // it relies on the navX reading in order to be field-centric
    private val diagonal: Distance = sqrt((wheelBase.inUnit(meters) * wheelBase.inUnit(meters) + trackWidth.inUnit(meters) * trackWidth.inUnit(meters))).meters

    // wheel radius is wheelDiameter / 2.
    private val wheelTravelPerMotorRadian: Distance = gearRatio * (wheelDiameter / 2)


    /*
    Encoder-based functions below
     */

    private val distanceOffset: Distance = overallEncoder.angularPosition * wheelTravelPerMotorRadian
    public val distanceTraveled: Distance
        get() =
            (overallEncoder.angularPosition *
                    wheelTravelPerMotorRadian) - distanceOffset
    public val velocity: Velocity
        get() =
            overallEncoder.angularVelocity *
                    wheelTravelPerMotorRadian





    public val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(
            UnitTranslation2d(trackWidth/2,wheelBase/2).inUnit(meters),
            UnitTranslation2d(trackWidth/2,-wheelBase/2).inUnit(meters),
            UnitTranslation2d(-trackWidth/2,wheelBase/2).inUnit(meters),
            UnitTranslation2d(-trackWidth/2,-wheelBase/2).inUnit(meters)
        )



    public val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyro.heading.asRotation2d(),
        arrayOf(
            topLeft.getModulePosition(gearRatio,wheelDiameter),
            topRight.getModulePosition(gearRatio,wheelDiameter),
            bottomLeft.getModulePosition(gearRatio,wheelDiameter),
            bottomRight.getModulePosition(gearRatio,wheelDiameter)
        ),
        startingPose.inUnit(meters)
    )

    override val robotPoseMeasurement: Measurement<UnitPose2d>
        get() = Measurement(
            poseEstimator.estimatedPosition.ofUnit(meters),
            Timer.getFPGATimestamp(),
            true
        )

    override val poseStandardDeviation: StandardDeviation = StandardDeviation.Default


    // note: pose2d's 2 important parameters here are called x and y.
    // thus, these 2 getters use those when getting the x coordinate and y coordinate, respectively.
    // note 2: these values, along with pose, are nullable.
    // They are set to null if the limelight has not detected any apriltags yet.
    public val xCoordinate: Distance
        get() = poseEstimator.estimatedPosition.x.meters
    public val yCoordinate: Distance
        get() = poseEstimator.estimatedPosition.y.meters

    public fun resetPose(pose: UnitPose2d, gyroAngle: Angle){
        poseEstimator.resetPosition(
            gyroAngle.asRotation2d(),
            currentModulePositions,
            pose.inUnit(meters)
        )
    }

    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPosition(
            gyro.heading.asRotation2d(),
            currentModulePositions,
            pose.inUnit(meters)
        )
    }


    public var currentModuleStates: Array<SwerveModuleState>
        get() = arrayOf(
            topLeft.getModuleState(gearRatio,wheelDiameter),
            topRight.getModuleState(gearRatio,wheelDiameter),
            bottomLeft.getModuleState(gearRatio,wheelDiameter),
            bottomRight.getModuleState(gearRatio,wheelDiameter)
        )
        set(moduleStates){
            topLeft.setDesiredState(moduleStates[0],gearRatio,wheelDiameter)
            topRight.setDesiredState(moduleStates[1],gearRatio,wheelDiameter)
            bottomLeft.setDesiredState(moduleStates[2],gearRatio,wheelDiameter)
            bottomRight.setDesiredState(moduleStates[3],gearRatio,wheelDiameter)
        }

    public val currentModulePositions: Array<SwerveModulePosition>
        get() = arrayOf(
            topLeft.getModulePosition(gearRatio,wheelDiameter),
            topRight.getModulePosition(gearRatio,wheelDiameter),
            bottomLeft.getModulePosition(gearRatio,wheelDiameter),
            bottomRight.getModulePosition(gearRatio,wheelDiameter)
        )



    /**
     * A variant of the swerveDrive function which usses [ChassisPowers] instead.
     */
    public fun swerveDrive(powers: ChassisPowers): Unit = swerveDrive(
        powers.xPower,
        powers.yPower,
        powers.rotationPower
    )

    /**
     * The default drive function. Makes the drivetrain move, using an xPower, yPower and rotationPower.
     */
    public fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double){

        val powers = if(fieldRelativeDrive){
            FieldRelativeChassisPowers(
                xPower,
                yPower,
                rotationPower,
                gyro.heading
            ).correctForDynamics()
        }else{
            ChassisPowers(
                xPower,
                yPower,
                rotationPower
            ).correctForDynamics()
        }

        val A: Double = powers.yPower - powers.rotationPower * (wheelBase.inUnit(meters)/diagonal.inUnit(meters))
        val B: Double = powers.yPower + powers.rotationPower * (wheelBase.inUnit(meters)/diagonal.inUnit(meters))
        val C: Double = powers.xPower - powers.rotationPower * (trackWidth.inUnit(meters)/diagonal.inUnit(meters))
        val D: Double = powers.xPower + powers.rotationPower * (trackWidth.inUnit(meters)/diagonal.inUnit(meters))

        var topRightPower: Double = sqrt(B*B + C*C)
        var topLeftPower: Double = sqrt(B*B+D*D)
        var bottomLeftPower: Double = sqrt(A*A+D*D)
        var bottomRightPower: Double = sqrt(A*A+C*C)


        // the following "normalizes" the wheel
        val max: Double? = listOf(topRightPower,topLeftPower,bottomLeftPower,bottomRightPower).maxOrNull()


        if (max != null && max > 1.0){
            topRightPower /= max
            topLeftPower /= max
            bottomRightPower /= max
            bottomLeftPower /= max
        }


        val topRightAngle: Angle = atan(B/C)
        val topLeftAngle: Angle = atan(B/D)
        val bottomLeftAngle: Angle = atan(A/D)
        val bottomRightAngle: Angle = atan(A/C)


        topLeft.setDirectionalPower(topLeftPower,topLeftAngle)
        topRight.setDirectionalPower(topRightPower,topRightAngle)
        bottomLeft.setDirectionalPower(bottomLeftPower,bottomLeftAngle)
        bottomRight.setDirectionalPower(bottomRightPower,bottomRightAngle)
    }

    /**
     * A version of velocityDrive that uses [ChassisSpeeds] instead.
     */
    public fun velocityDrive(speeds: ChassisSpeeds): Unit = velocityDrive(
        speeds.xVelocity,
        speeds.yVelocity,
        speeds.rotationSpeed
    )

    /**
     * Drives the robot using certain velocities instead of certain amounts of power(or duty cycle output).
     * Relies on PID in order to be used effectively.
     */
    public fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationVelocity: AngularVelocity){


        val speeds = if(fieldRelativeDrive){
            FieldRelativeChassisSpeeds(
                xVelocity,
                yVelocity,
                rotationVelocity,
                gyro.heading
            ).correctForDynamics()
        }else{
            ChassisSpeeds(
                xVelocity,
                yVelocity,
                rotationVelocity
            ).correctForDynamics()
        }

        currentModuleStates = kinematics.toSwerveModuleStates(speeds)
    }

    /**
    * Drives the drivetrain with a specific power at a specified angle.
     */
    public fun directionalDrive(xPower: Double, angle: Angle){
        topLeft.setDirectionalPower(xPower,angle)
        topRight.setDirectionalPower(xPower,angle)
        bottomLeft.setDirectionalPower(xPower,angle)
        bottomRight.setDirectionalPower(xPower,angle)
    }

    /**
    * Stops the drivetrain.
     */
    public fun stop(){
        topLeft.halt()
        topRight.halt()
        bottomLeft.halt()
        bottomRight.halt()
    }

    /**
    * Makes the drivetrain rotate in place, with a specific power.
     */
    public fun rotateInPlace(power: Double) {
        topLeft.setDirectionalPower(power,45.degrees)
        topRight.setDirectionalPower(power, (-45).degrees)
        bottomLeft.setDirectionalPower(power,45.degrees)
        bottomRight.setDirectionalPower(power, (-45).degrees)
    }

    /**
     * adds a variable amount of pose suppliers to the drivetrain.
     * these pose suppliers will fuse their poses into the pose estimator for more accurate measurements.
     */
    public fun addPoseSuppliers(vararg poseSuppliers: RobotPoseSupplier){
        allPoseSuppliers.addAll(poseSuppliers)
    }

    override fun periodic() {
        poseEstimator.update(
            gyro.heading.asRotation2d(),
            currentModulePositions
        )

        allPoseSuppliers.forEach{
            val poseMeasurement = it.robotPoseMeasurement

            it.poseStandardDeviation.processValue(
                whenValueExists = { stdDev ->
                    poseEstimator.addVisionMeasurement(
                        poseMeasurement.value.inUnit(meters),
                        poseMeasurement.timestamp.inUnit(seconds),
                        stdDev.getVector()
                    )
                },
                whenDefault = {
                    poseEstimator.addVisionMeasurement(
                        poseMeasurement.value.inUnit(meters),
                        poseMeasurement.timestamp.inUnit(seconds)
                    )
                }
            )
        }

    }


}