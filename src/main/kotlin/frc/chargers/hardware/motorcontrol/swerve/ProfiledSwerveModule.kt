package frc.chargers.hardware.motorcontrol.swerve

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.AngularProfiledPIDController
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.Precision
import frc.chargers.utils.math.units.rem
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import frc.chargers.wpilibextensions.kinematics.swerve.SwerveModulePosition
import frc.chargers.wpilibextensions.kinematics.swerve.SwerveModuleState

/**
 * Represents a basic swerve module w/ configuration.
 *
 * @see NonConfigurableSwerveModule
 */
public class ProfiledSwerveModule<TMC: MotorConfiguration, DMC: MotorConfiguration> private constructor(
    turnMotor: EncoderMotorController,
    turnEncoder: PositionEncoder? = null,
    driveMotor: EncoderMotorController,
    turnPIDConstants: PIDConstants,
    turnFF: AngularMotorFF = AngularMotorFF.None,
    profileConstraints: AngularTrapezoidProfile.Constraints,
    drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    velocityFF: AngularMotorFF = AngularMotorFF.None,
    turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    useOnboardPIDIfAvailable: Boolean = false
): NonConfigurableProfiledSwerveModule(
    turnMotor,
    turnEncoder,
    driveMotor,
    turnPIDConstants,
    turnFF,
    profileConstraints,
    drivePIDConstants,
    velocityFF,
    turnPrecision,
    useOnboardPIDIfAvailable
), HolonomicModule<TMC, DMC> {

    public companion object{

        public operator fun <TM, TMC: MotorConfiguration, DM, DMC: MotorConfiguration>invoke(
            turnMotor: TM,
            turnEncoder: PositionEncoder? = null,
            driveMotor: DM,
            data: Data,
            configuration: ModuleConfiguration<TMC, DMC>
        ): ProfiledSwerveModule<TMC, DMC> where TM: MotorConfigurable<TMC>, TM: EncoderMotorController, DM: MotorConfigurable<DMC>, DM: EncoderMotorController =
        invoke(
            turnMotor,
            turnEncoder,
            driveMotor,
            data.turnPIDConstants,
            data.turnFF,
            data.profileConstraints,
            data.drivePIDConstants,
            data.velocityFF,
            data.turnPrecision,
            data.useOnboardPIDIfAvailable,
            configuration
        )


        public operator fun <TM, TMC: MotorConfiguration, DM, DMC: MotorConfiguration>invoke(
            turnMotor: TM,
            turnEncoder: PositionEncoder? = null,
            driveMotor: DM,
            turnPIDConstants: PIDConstants,
            turnFF: AngularMotorFF = AngularMotorFF.None,
            profileConstraints: AngularTrapezoidProfile.Constraints,
            drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
            velocityFF: AngularMotorFF = AngularMotorFF.None,
            turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
            useOnboardPIDIfAvailable: Boolean = false,
            configuration: ModuleConfiguration<TMC, DMC>? = null
        ): ProfiledSwerveModule<TMC, DMC> where TM: MotorConfigurable<TMC>, TM: EncoderMotorController, DM: MotorConfigurable<DMC>, DM: EncoderMotorController =
            ProfiledSwerveModule(
                turnMotor.apply{
                    if(configuration != null){
                        configure(configuration.turnMotorConfig)
                    }
                },
                turnEncoder,
                driveMotor.apply{
                    if(configuration != null){
                        configure(configuration.driveMotorConfig)
                    }
                },
                turnPIDConstants,
                turnFF,
                profileConstraints,
                drivePIDConstants,
                velocityFF,
                turnPrecision,
                useOnboardPIDIfAvailable
            )
    }



    public data class Data(
        val turnPIDConstants: PIDConstants,
        val turnFF: AngularMotorFF = AngularMotorFF.None,
        val profileConstraints: AngularTrapezoidProfile.Constraints,
        val drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
        val velocityFF: AngularMotorFF = AngularMotorFF.None,
        val turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        val useOnboardPIDIfAvailable: Boolean = false
    )

    override fun configure(configuration: ModuleConfiguration<TMC, DMC>) {
        @Suppress("UNCHECKED_CAST")
        driveMotor as MotorConfigurable<DMC>
        driveMotor.configure(configuration.driveMotorConfig)

        @Suppress("UNCHECKED_CAST")
        turnMotor as MotorConfigurable<TMC>
        turnMotor.configure(configuration.turnMotorConfig)
    }

}

/**
 * Represents a Swerve Module without motor configuration.
 *
 */
public open class NonConfigurableProfiledSwerveModule(
    public val turnMotor: EncoderMotorController,
    public val turnEncoder: PositionEncoder? = null,
    public val driveMotor: EncoderMotorController,
    public val turnPIDConstants: PIDConstants,
    public val turnFF: AngularMotorFF = AngularMotorFF.None,
    public val profileConstraints: AngularTrapezoidProfile.Constraints,
    public val drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    public val velocityFF: AngularMotorFF = AngularMotorFF.None,
    public val turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    useOnboardPIDIfAvailable: Boolean = false,
): NonConfigurableHolonomicModule {

    override val distanceMeasurementEncoder: Encoder = driveMotor.encoder


    private val currentDirection: Angle
        get() = turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition

    /**
     * A base lambda for the SwerveModule class.
     */
    private val turnMotorPositionSetter: (Angle) -> Unit = if(useOnboardPIDIfAvailable && turnMotor is FeedbackMotorController){

        // turnMotor smart casts to FeedbackMotorController here, which means that setAngularPosition is allowed.

        if (turnPrecision is Precision.Within){
            // return value
            {
                if(currentDirection-it !in turnPrecision.allowableError) {
                    turnMotor.setAngularPosition(
                        it,
                        turnPIDConstants,
                        turnFF,
                        profileConstraints,
                        turnEncoder
                    )
                }else{
                    turnMotor.set(0.0)
                }
            }
        }else{
            // return value
            {
                turnMotor.setAngularPosition(
                    it,
                    turnPIDConstants,
                    turnFF,
                    profileConstraints,
                    turnEncoder
                )
            }
        }

    }else{
        val controller = AngularProfiledPIDController(
            pidConstants = turnPIDConstants,
            getInput = if(turnEncoder == null){
                {turnMotor.encoder.angularPosition}
            }else{
                {turnEncoder.angularPosition}
            },
            target = Angle(0.0),
            constraints = profileConstraints,
            feedforward = turnFF,
            selfSustain = true
        )
        // Needed so that the return lambda isn't confused to be part of the superPIDController

        if(turnPrecision is Precision.Within){
            // return value
            {
                if(controller.target != it){controller.target = it}
                if (currentDirection-it !in turnPrecision.allowableError){
                    turnMotor.setVoltage(controller.calculateOutput().inUnit(volts))
                }else{
                    turnMotor.set(0.0)
                }
            }
        }else{
            // return value
            {
                if(controller.target != it){controller.target = it}
                turnMotor.setVoltage(controller.calculateOutput().inUnit(volts))
            }
        }
    }

    /**
     * A base lambda for the SwerveModule class.
     */
    private val driveMotorVelocitySetter: (AngularVelocity) -> Unit = if(useOnboardPIDIfAvailable && driveMotor is FeedbackMotorController){
        // return value
        {
            driveMotor.setAngularVelocity(
                it,
                drivePIDConstants,
                velocityFF
            )
        }
    }else{
        val controller = UnitSuperPIDController(
            pidConstants = drivePIDConstants,
            getInput = {driveMotor.encoder.angularVelocity},
            target = AngularVelocity(0.0),
            selfSustain = true,
            feedforward = velocityFF
        );
        // return value
        {
            if(controller.target != it){controller.target = it}
            driveMotor.setVoltage(controller.calculateOutput().inUnit(volts))
        }
    }

    override fun setDirectionalPower(power: Double, direction: Angle) {
        if (abs(direction - currentDirection) > 90.0.degrees){
            driveMotor.set(-power)
            turnMotorPositionSetter((direction + 180.degrees) % 360.degrees)
        }else{
            driveMotor.set(power)
            turnMotorPositionSetter(direction)
        }
    }

    override fun setDirectionalVelocity(angularVelocity: AngularVelocity, direction: Angle) {
        if (abs(direction - currentDirection) > 90.0.degrees){
            driveMotorVelocitySetter(-angularVelocity)
            turnMotorPositionSetter((direction + 180.degrees) % 360.degrees)
        }else{
            driveMotorVelocitySetter(angularVelocity)
            turnMotorPositionSetter(direction)
        }
    }

    override fun halt(){
        driveMotor.set(0.0)
        turnMotor.set(0.0)
    }

    override fun getModuleState(gearRatio: Double, wheelDiameter: Length): SwerveModuleState =
        SwerveModuleState(
            driveMotor.encoder.angularVelocity * (gearRatio * wheelDiameter),
            currentDirection
        )

    override fun getModulePosition(gearRatio: Double, wheelDiameter: Length): SwerveModulePosition =
        SwerveModulePosition(
            driveMotor.encoder.angularPosition * (gearRatio * wheelDiameter),
            currentDirection
        )

}
