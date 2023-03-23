package frc.chargers.hardware.subsystems.drivetrain


import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.Degrees
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.SwerveModule
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import frc.chargers.hardware.sensors.encoders.Encoder
import kotlin.math.*


/*
An implementation of Swerve drive, with encoders, to be used in future robot code.
Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
* */

// note: default gear ratio defined in EncoderDifferentialDriveTrain.
// They're in the same package and thus no import
// note: trackwidth is horizontal and wheelBase is vertical
// second note: DEFAULT_GEAR_RATIO is defined in encoderdifferentialdrivetrain.
public class EncoderHolonomicDrivetrain(private val topLeft: SwerveModule,
                                        private val topRight: SwerveModule,
                                        private val bottomLeft: SwerveModule,
                                        private val bottomRight: SwerveModule,
                                        protected val gyro: NavX? = null,
                                        protected val gearRatio: Double = DEFAULT_GEAR_RATIO,
                                        protected val wheelDiameter: Length,
                                        protected val trackWidth: Distance,
                                        protected val wheelBase: Distance): SubsystemBase(){


    public val overallEncoder: Encoder = AverageEncoder(
        topLeft.driveEncoder,
        topRight.driveEncoder,
        bottomLeft.driveEncoder,
        bottomRight.driveEncoder)


    // all borrowed from EncoderDifferentialDriveTrain. see more information there.
    // note: an encoderHolonomicDriveTrain CANNOT provide heading. it relies on the navX reading in order to be field-centric
    private val diagonal: Distance = sqrt((wheelBase.inUnit(meters) * wheelBase.inUnit(meters) + trackWidth.inUnit(meters) * trackWidth.inUnit(meters))).meters

    private val wheelRadius = wheelDiameter / 2
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

    private var driveFunctionCalled: Boolean = false

    // counter-intuitively, xPower is straight, and yPower is side-to-side.
    public fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double){
        // a gyro input into the swerveDrive allows it to remain field-centric; optional


        val forwardPower: Double = if(gyro == null){xPower}else{
            xPower*cos(gyro!!.gyroscope.heading.inUnit(Degrees)) + yPower * sin(gyro!!.gyroscope.heading.inUnit(Degrees))
        }
        val sidePower: Double = if(gyro == null){yPower}else{
            -xPower * sin(gyro!!.gyroscope.heading.inUnit(Degrees)) + yPower * cos(gyro!!.gyroscope.heading.inUnit(Degrees))
        }




        var A: Double = sidePower - rotationPower * (wheelBase.inUnit(meters)/diagonal.inUnit(meters))
        var B: Double = sidePower + rotationPower * (wheelBase.inUnit(meters)/diagonal.inUnit(meters))
        var C: Double = forwardPower - rotationPower * (trackWidth.inUnit(meters)/diagonal.inUnit(meters))
        var D: Double = forwardPower + rotationPower * (trackWidth.inUnit(meters)/diagonal.inUnit(meters))

        var topRightPower: Double = sqrt(B*B + C*C)
        var topLeftPower: Double = sqrt(B*B+D*D)
        var bottomLeftPower: Double = sqrt(A*A+D*D)
        var bottomRightPower: Double = sqrt(A*A+C*C)

        // note: radians doesn't work. rohen pls fix
        var topRightAngle: Angle = atan(B/C)*(180/PI).degrees
        var topLeftAngle: Angle = atan(B/D)*(180/PI).degrees
        var bottomRightAngle: Angle = atan(A/D)*(180/PI).degrees
        var bottomLeftAngle: Angle = atan(A/C)*(180/PI).degrees


        topLeft.setDirectionalPower(topLeftAngle,topLeftPower)
        topRight.setDirectionalPower(topRightAngle,topRightPower)
        bottomLeft.setDirectionalPower(bottomLeftAngle,bottomLeftPower)
        bottomRight.setDirectionalPower(bottomRightAngle,bottomRightPower)

        driveFunctionCalled = true

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