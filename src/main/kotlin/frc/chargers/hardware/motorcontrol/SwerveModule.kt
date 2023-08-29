package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.kinematics.SwerveModuleState
import frc.chargers.wpilibextensions.kinematics.SwerveModulePosition


/**
 * Represents a basic swerve module w/ configuration.
 *
 * @see NonConfigurableSwerveModule
 */
public class SwerveModule<TMC: MotorConfiguration, DMC: MotorConfiguration> private constructor(
    turnMotor: EncoderMotorController,
    turnEncoder: Encoder? = null,
    driveMotor: EncoderMotorController,
    turnPIDConstants: PIDConstants,
    drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    velocityFF: AngularMotorFF = AngularMotorFF.None,
    turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    useOnboardPIDIfAvailable: Boolean = false
): NonConfigurableSwerveModule(
    turnMotor,
    turnEncoder,
    driveMotor,
    turnPIDConstants,
    drivePIDConstants,
    velocityFF,
    turnPrecision,
    useOnboardPIDIfAvailable
), HolonomicModule<TMC,DMC>{

    public companion object{

        public operator fun <TM, TMC: MotorConfiguration, DM, DMC: MotorConfiguration>invoke(
            turnMotor: TM,
            turnEncoder: Encoder? = null,
            driveMotor: DM,
            data: Data,
            turnMotorConfiguration: TMC? = null,
            driveMotorConfiguration: DMC? = null
        ): SwerveModule<TMC,DMC> where TM: MotorConfigurable<TMC>, TM: EncoderMotorController, DM: MotorConfigurable<DMC>, DM: EncoderMotorController =
            invoke(
                turnMotor,
                turnEncoder,
                driveMotor,
                data.turnPIDConstants,
                data.drivePIDConstants,
                data.velocityFF,
                data.turnPrecision,
                data.useOnboardPIDIfAvailable,
                turnMotorConfiguration,
                driveMotorConfiguration
            )
        public operator fun <TM, TMC: MotorConfiguration, DM, DMC: MotorConfiguration>invoke(
            turnMotor: TM,
            turnEncoder: Encoder? = null,
            driveMotor: DM,
            turnPIDConstants: PIDConstants,
            drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
            velocityFF: AngularMotorFF = AngularMotorFF.None,
            turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
            useOnboardPIDIfAvailable: Boolean = false,
            turnMotorConfiguration: TMC? = null,
            driveMotorConfiguration: DMC? = null
        ): SwerveModule<TMC,DMC> where TM: MotorConfigurable<TMC>, TM: EncoderMotorController, DM: MotorConfigurable<DMC>, DM: EncoderMotorController =
            SwerveModule(
            turnMotor.apply{
                if(turnMotorConfiguration != null){
                    configure(turnMotorConfiguration)
                }
            },
            turnEncoder,
            driveMotor.apply{
                if(driveMotorConfiguration != null){
                    configure(driveMotorConfiguration)
                }
            },
            turnPIDConstants,
            drivePIDConstants,
            velocityFF,
            turnPrecision,
            useOnboardPIDIfAvailable
        )
    }

    override fun configureDriveMotor(configuration: DMC) {
        @Suppress("UNCHECKED_CAST")
        driveMotor as MotorConfigurable<DMC>
        driveMotor.configure(configuration)
    }

    override fun configureTurnMotor(configuration: TMC) {
        @Suppress("UNCHECKED_CAST")
        turnMotor as MotorConfigurable<TMC>
        turnMotor.configure(configuration)
    }

    public data class Data(
        val turnPIDConstants: PIDConstants,
        val drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
        val velocityFF: AngularMotorFF = AngularMotorFF.None,
        val turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
        val useOnboardPIDIfAvailable: Boolean = false
    )

}

/**
 * Represents a Swerve Module without motor configuration.
 *
 */
public open class NonConfigurableSwerveModule(
    public val turnMotor: EncoderMotorController,
    public val turnEncoder: Encoder? = null,
    public val driveMotor: EncoderMotorController,
    public val turnPIDConstants: PIDConstants,
    public val drivePIDConstants: PIDConstants = PIDConstants(0.0,0.0,0.0),
    public val velocityFF: AngularMotorFF = AngularMotorFF.None,
    public val turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    useOnboardPIDIfAvailable: Boolean = false
): NonConfigurableHolonomicModule{

    override val distanceMeasurementEncoder: Encoder = driveMotor.encoder


    /**
     * A base lambda for the SwerveModule class.
     */
    private val turnMotorPositionSetter: (Angle) -> Unit = if(useOnboardPIDIfAvailable && turnMotor is FeedbackMotorController){
        // return value
        // turnMotor smart casts to FeedbackMotorController here, which means that setAngularPosition is allowed.
        {
            turnMotor.setAngularPosition(
                it,
                turnPIDConstants,
                turnEncoder
            )
        }
    }else{
        val controller = UnitSuperPIDController<AngleDimension,VoltageDimension>(
            pidConstants = turnPIDConstants,
            getInput = if(turnEncoder == null){
                {turnMotor.encoder.angularPosition}
            }else{
                {turnEncoder.angularPosition}
            },
            target = Angle(0.0),
            selfSustain = true
        )
        // Needed so that the return lambda isn't confused to be part of the superPIDController
        val separator = Unit

        // return value
        {
            if(controller.target != it){controller.target = it}
            turnMotor.setVoltage(controller.calculateOutput().inUnit(volts))
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
        )
        // separator is necessary to prevent the compiler
        // from thinking the return lambda is part of the UnitSuperPIDController
        val separator = Unit
        // return value
        {
            if(controller.target != it){controller.target = it}
            driveMotor.setVoltage(controller.calculateOutput().inUnit(volts))
        }
    }

    override fun setDirectionalPower(power: Double, direction: Angle) {
        val delta = direction - (turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition)
        if (abs(delta) > 90.0.degrees){
            val newDirection = ((direction.inUnit(degrees) + 180) % 360).ofUnit(degrees)
            driveMotor.set(-power)
            when(turnPrecision){
                Precision.AllowOvershoot -> turnMotorPositionSetter(newDirection)

                is Precision.Within -> {
                    if((turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition)-newDirection in turnPrecision.allowableError){
                        turnMotorPositionSetter(newDirection)
                    }else{
                        turnMotor.set(0.0)
                    }
                }
            }

        }else{
            driveMotor.set(power)
            when(turnPrecision){
                Precision.AllowOvershoot -> turnMotorPositionSetter(direction)

                is Precision.Within -> {
                    if((turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition)-direction in turnPrecision.allowableError){
                        turnMotorPositionSetter(direction)
                    }else{
                        turnMotor.set(0.0)
                    }
                }
            }
        }
    }

    override fun setDirectionalVelocity(angularVelocity: AngularVelocity, direction: Angle) {
        val delta = direction - (turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition)
        if (abs(delta) > 90.0.degrees){
            val newDirection = ((direction.inUnit(degrees) + 180) % 360).ofUnit(degrees)
            driveMotorVelocitySetter(-angularVelocity)
            when(turnPrecision){
                Precision.AllowOvershoot -> turnMotorPositionSetter(newDirection)

                is Precision.Within -> {
                    if((turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition)-newDirection in turnPrecision.allowableError){
                        turnMotorPositionSetter(newDirection)
                    }else{
                        turnMotor.set(0.0)
                    }
                }
            }
        }else{
            driveMotorVelocitySetter(angularVelocity)
            when(turnPrecision){
                Precision.AllowOvershoot -> turnMotorPositionSetter(direction)

                is Precision.Within -> {
                    if((turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition)-direction in turnPrecision.allowableError){
                        turnMotorPositionSetter(direction)
                    }else{
                        turnMotor.set(0.0)
                    }
                }
            }
        }
    }

    override fun halt(){
        driveMotor.set(0.0)
        turnMotor.set(0.0)
    }

    override fun getModuleState(gearRatio: Double, wheelDiameter: Length): SwerveModuleState =
        SwerveModuleState(
            driveMotor.encoder.angularVelocity * (gearRatio * wheelDiameter),
            turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition
        )

    override fun getModulePosition(gearRatio: Double, wheelDiameter: Length): SwerveModulePosition =
        SwerveModulePosition(
            driveMotor.encoder.angularPosition * (gearRatio * wheelDiameter),
            turnEncoder?.angularPosition ?: turnMotor.encoder.angularPosition
        )

}
