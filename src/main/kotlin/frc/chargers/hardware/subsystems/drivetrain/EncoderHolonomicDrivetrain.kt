package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
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
import frc.chargers.wpilibextensions.kinematics.swerve.ModulePositions
import frc.chargers.wpilibextensions.kinematics.swerve.ModuleSpeeds
import frc.chargers.wpilibextensions.kinematics.swerve.SuperSwerveKinematics

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





    /*
    public val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(
            UnitTranslation2d(trackWidth/2,wheelBase/2).inUnit(meters),
            UnitTranslation2d(trackWidth/2,-wheelBase/2).inUnit(meters),
            UnitTranslation2d(-trackWidth/2,wheelBase/2).inUnit(meters),
            UnitTranslation2d(-trackWidth/2,-wheelBase/2).inUnit(meters)
        )
     */


    public val kinematics: SuperSwerveKinematics = SuperSwerveKinematics(
        UnitTranslation2d(trackWidth/2,wheelBase/2),
        UnitTranslation2d(trackWidth/2,-wheelBase/2),
        UnitTranslation2d(-trackWidth/2,wheelBase/2),
        UnitTranslation2d(-trackWidth/2,-wheelBase/2)
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


    public fun resetPose(pose: UnitPose2d, gyroAngle: Angle){
        poseEstimator.resetPosition(
            gyroAngle.asRotation2d(),
            currentModulePositions.toArray(),
            pose.inUnit(meters)
        )
    }

    public fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPosition(
            gyro.heading.asRotation2d(),
            currentModulePositions.toArray(),
            pose.inUnit(meters)
        )
    }



    public var currentModuleStates: ModuleSpeeds
        get() = ModuleSpeeds(
            topLeftState = topLeft.getModuleState(gearRatio,wheelDiameter),
            topRightState = topRight.getModuleState(gearRatio,wheelDiameter),
            bottomLeftState = bottomLeft.getModuleState(gearRatio,wheelDiameter),
            bottomRightState = bottomRight.getModuleState(gearRatio,wheelDiameter)
        )
        set(ms){
            topLeft.setDirectionalVelocity(ms.topLeftSpeed,ms.topLeftAngle,gearRatio,wheelDiameter)
            topRight.setDirectionalVelocity(ms.topRightSpeed,ms.topRightAngle,gearRatio,wheelDiameter)
            bottomLeft.setDirectionalVelocity(ms.bottomLeftSpeed,ms.bottomLeftAngle,gearRatio,wheelDiameter)
            bottomRight.setDirectionalVelocity(ms.bottomRightSpeed,ms.bottomRightAngle,gearRatio,wheelDiameter)
        }



    public val currentModulePositions: ModulePositions
        get() = ModulePositions(
            topLeftPosition = topLeft.getModulePosition(gearRatio,wheelDiameter),
            topRightPosition = topRight.getModulePosition(gearRatio,wheelDiameter),
            bottomLeftPosition = bottomLeft.getModulePosition(gearRatio,wheelDiameter),
            bottomRightPosition = bottomRight.getModulePosition(gearRatio,wheelDiameter)
        )


    
    

    /**
     * A version of swerveDrive that uses [ChassisSpeeds] instead.
     */
    public fun swerveDrive(speeds: ChassisSpeeds): Unit = swerveDrive(
        speeds.xVelocity,
        speeds.yVelocity,
        speeds.rotationSpeed
    )

    /**
     * Drives the robot using certain velocities instead of certain amounts of power(or duty cycle output).
     * Relies on PID in order to be used effectively.
     */
    public fun swerveDrive(xVelocity: Velocity, yVelocity: Velocity, rotationVelocity: AngularVelocity){

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

        currentModuleStates = kinematics.toFirstOrderModuleSpeeds(speeds)
        // currentModuleStates = kinematics.toSecondOrderModuleSpeeds(speeds,gyro.heading)
    }

    /**
    * Drives the drivetrain with a specific speed at a specified angle.
     */
    public fun directionalDrive(speed: Velocity, angle: Angle){
        topLeft.setDirectionalVelocity(speed,angle,gearRatio, wheelDiameter)
        topRight.setDirectionalVelocity(speed,angle,gearRatio,wheelDiameter)
        bottomLeft.setDirectionalVelocity(speed,angle,gearRatio,wheelDiameter)
        bottomRight.setDirectionalVelocity(speed,angle,gearRatio,wheelDiameter)
    }

    /**
     * Drives the drivetrain with a specific speed at a specified angle.
     */
    public fun directionalDrive(power: Double, angle: Angle){
        topLeft.setDirectionalPower(power,angle)
        topRight.setDirectionalPower(power,angle)
        bottomLeft.setDirectionalPower(power,angle)
        bottomRight.setDirectionalPower(power,angle)
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
     * Stops the drivetrain in an X.
     */
    public fun stopInX(){
        topLeft.setDirectionalPower(0.0,-45.degrees)
        topRight.setDirectionalPower(0.0,45.degrees)
        bottomLeft.setDirectionalPower(0.0,45.degrees)
        bottomRight.setDirectionalPower(0.0,-45.degrees)
    }

    /**
    * Makes the drivetrain rotate in place, with a specific power.
     */
    public fun rotateInPlace(speed: Velocity) {
        topLeft.setDirectionalVelocity(speed,45.degrees,gearRatio, wheelDiameter)
        topRight.setDirectionalVelocity(speed,-45.degrees,gearRatio,wheelDiameter)
        bottomLeft.setDirectionalVelocity(speed,-45.degrees,gearRatio,wheelDiameter)
        bottomRight.setDirectionalVelocity(speed,45.degrees,gearRatio,wheelDiameter)
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
            currentModulePositions.toArray()
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