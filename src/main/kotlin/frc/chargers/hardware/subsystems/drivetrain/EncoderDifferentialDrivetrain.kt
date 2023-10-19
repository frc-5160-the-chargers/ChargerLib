package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.MotorConfiguration
import frc.chargers.hardware.motorcontrol.ctre.TalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.utils.a
import org.littletonrobotics.junction.Logger

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

public fun simulatedDrivetrain(
    simMotors: DifferentialDrivetrainSim.KitbotMotor,
    loopPeriod: Time = 20.milli.seconds,
    invertMotors: Boolean = false,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
): EncoderDifferentialDrivetrain = EncoderDifferentialDrivetrain(
    DifferentialDriveIOSim(simMotors,loopPeriod),
    invertMotors, gearRatio, wheelDiameter, width,  leftVelocityConstants, leftMotorFF, rightVelocityConstants, rightMotorFF
)

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrain(
    leftMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    rightMotors: EncoderMotorControllerGroup<SparkMaxConfiguration>,
    invertMotors: Boolean = false, gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    configure: SparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, leftVelocityConstants, leftMotorFF , rightVelocityConstants, rightMotorFF,
        configuration = SparkMaxConfiguration().apply(configure))

/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrain(
    leftMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    rightMotors: EncoderMotorControllerGroup<TalonFXConfiguration>,
    invertMotors: Boolean = false,
    gearRatio: Double = DEFAULT_GEAR_RATIO,
    wheelDiameter: Length,
    width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    configure: TalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(leftMotors, rightMotors, invertMotors, gearRatio, wheelDiameter, width, leftVelocityConstants, leftMotorFF , rightVelocityConstants, rightMotorFF,
        configuration = TalonFXConfiguration().apply(configure))

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 */
public fun <C : MotorConfiguration> EncoderDifferentialDrivetrain(
    leftMotors: EncoderMotorControllerGroup<C>,
    rightMotors: EncoderMotorControllerGroup<C>,
    invertMotors: Boolean = false, gearRatio: Double,
    wheelDiameter: Length,
    width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
    configuration: C
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        io = DifferentialDriveIOReal(
            leftMotors = leftMotors.apply { configure(configuration) },
            rightMotors = rightMotors.apply { configure(configuration) }
        ),
        invertMotors = invertMotors,
        gearRatio = gearRatio,
        wheelDiameter = wheelDiameter,
        width = width,
        leftVelocityConstants, leftMotorFF, rightVelocityConstants, rightMotorFF
    )



public class EncoderDifferentialDrivetrain(
    private val io: DifferentialDriveIO,
    invertMotors: Boolean = false,
    private val gearRatio: Double = DEFAULT_GEAR_RATIO,
    private val wheelDiameter: Length,
    private val width: Distance,
    leftVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    leftMotorFF: AngularMotorFF = AngularMotorFF.None,
    rightVelocityConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    rightMotorFF: AngularMotorFF = AngularMotorFF.None,
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider{
    init{
        io.inverted = invertMotors
    }
    internal val inputs = DifferentialDriveIO.Inputs()


    override fun periodic(){
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Drivetrain(Differential)",inputs)
        
        leftController.calculateOutput()
        rightController.calculateOutput()

    }

    override fun tankDrive(leftPower: Double, rightPower: Double) {
        io.setVoltages(leftPower * 12.volts, rightPower * 12.volts)
    }

    override fun arcadeDrive(power: Double, rotation: Double) {
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(power,rotation,false)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    override fun curvatureDrive(power: Double, steering: Double) {
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(power,steering,true)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    override fun stop() {
        io.setVoltages(0.volts,0.volts)
    }


    private val wheelRadius = wheelDiameter / 2
    internal val wheelTravelPerMotorRadian = gearRatio * wheelRadius

    /**
     * The total linear distance traveled from the zero point of the encoders.
     *
     * This value by itself is not particularly meaningful as it may be fairly large,
     * positive or negative, based on previous rotations of the motors, including
     * from previous times the robot has been enabled.
     *
     * Thus, it's more common to use this property to determine *change* in position.
     * If the initial value of this property is stored, the distance traveled since
     * that initial point can easily be determined by subtracting the initial position
     * from the current position.
     */
    public val distanceTraveled: Distance
        get() =
            a[inputs.leftAngularPosition,inputs.rightAngularPosition].average() * wheelTravelPerMotorRadian

    /**
     * The current linear velocity of the robot.
     */
    public val velocity: Velocity
        get() =
            a[inputs.leftAngularVelocity,inputs.rightAngularVelocity].average() * wheelTravelPerMotorRadian

    /**
     * The current heading (the direction the robot is facing).
     *
     * This value is calculated using the encoders, not a gyroscope or accelerometer,
     * so note that it may become inaccurate if the wheels slip. If available, consider
     * using a [frc.chargers.hardware.sensors.NavX] or similar device to calculate heading instead.
     *
     * This value by itself is not particularly meaningful as it may be fairly large,
     * positive or negative, based on previous rotations of the motors, including
     * from previous times the robot has been enabled.
     *
     * Thus, it's more common to use this property to determine *change* in heading.
     * If the initial value of this property is stored, the amount of rotation since
     * that initial point can easily be determined by subtracting the initial heading
     * from the current heading.
     *
     * @see HeadingProvider
     */
    public override val heading: Angle
        get() = wheelTravelPerMotorRadian *
                (inputs.rightAngularPosition - inputs.leftAngularPosition) / width


    /**
     * The kinematics of the drivetrain.
     *
     * @see DifferentialDriveKinematics
     */
    public val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
        width.inUnit(meters)
    )

    private val leftController = UnitSuperPIDController(
        leftVelocityConstants,
        {inputs.leftAngularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = false,
        feedforward = leftMotorFF
    )

    private val rightController = UnitSuperPIDController(
        rightVelocityConstants,
        {inputs.rightAngularVelocity},
        target = AngularVelocity(0.0),
        selfSustain = false,
        feedforward = rightMotorFF
    )

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity){
        leftController.target = leftSpeed / (gearRatio * wheelDiameter)
        rightController.target = rightSpeed / (gearRatio * wheelDiameter)
        io.setVoltages(
            left = leftController.calculateOutput(),
            right = rightController.calculateOutput()
        )
    }

    public fun velocityDrive(speeds: ChassisSpeeds){
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        velocityDrive(
            wheelSpeeds.leftMetersPerSecond.ofUnit(meters/seconds),
            wheelSpeeds.rightMetersPerSecond.ofUnit(meters/seconds)
        )
    }






    

}