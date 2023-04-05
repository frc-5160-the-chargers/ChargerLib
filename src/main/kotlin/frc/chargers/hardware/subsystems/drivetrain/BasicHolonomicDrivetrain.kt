package frc.chargers.hardware.subsystems.drivetrain


import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.SwerveModule
import frc.chargers.hardware.sensors.Limelight
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import frc.chargers.hardware.sensors.encoders.Encoder
import kotlin.math.sin
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.PI


/*
An implementation of Swerve drive, with encoders, to be used in future robot code.
Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
* */

// note: default gear ratio defined in EncoderDifferentialDriveTrain.
// They're in the same package and thus no import
// note: trackwidth is horizontal and wheelBase is vertical
// second note: DEFAULT_GEAR_RATIO is defined in encoderdifferentialdrivetrain.
public open class BasicHolonomicDrivetrain(
    private val topLeft: SwerveModule,
    private val topRight: SwerveModule,
    private val bottomLeft: SwerveModule,
    private val bottomRight: SwerveModule,
    private val gyro: NavX,
    private val gearRatio: Double = DEFAULT_GEAR_RATIO,
    private val wheelDiameter: Length,
    private val trackWidth: Distance,
    private val wheelBase: Distance
): SubsystemBase(), HolonomicDrivetrain{
    // do drivetrain.fieldRelativeDrive = false to turn this option off.
    public var fieldRelativeDrive: Boolean = true



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
    private val wheelTravelPerMotorRadian = gearRatio * wheelRadius

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

    override fun directionalDrive(xPower: Double, angle: Angle){
        topLeft.setDirectionalPower(angle,xPower)
        topRight.setDirectionalPower(angle,xPower)
        bottomLeft.setDirectionalPower(angle,xPower)
        bottomRight.setDirectionalPower(angle,xPower)
    }

    override fun stop(){
        topLeft.halt()
        topRight.halt()
        bottomLeft.halt()
        bottomRight.halt()
    }

    override fun rotateInPlace(power: Double) {
        topLeft.setDirectionalPower(45.degrees,power)
        topRight.setDirectionalPower(-(45.degrees),power)
        bottomLeft.setDirectionalPower(45.degrees,power)
        bottomRight.setDirectionalPower(-(45.degrees),power)
    }



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

}



}