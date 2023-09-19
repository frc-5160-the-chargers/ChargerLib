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
 * Represents a profiled swerve module w/ configuration.
 *
 * @see NonConfigurableSwerveModule
 */
public class ProfiledSwerveModule<TMC: MotorConfiguration, DMC: MotorConfiguration> private constructor(
    turnMotor: EncoderMotorController,
    turnEncoder: PositionEncoder,
    driveMotor: EncoderMotorController,
    turnPIDConstants: PIDConstants,
    turnFF: AngularMotorFF = AngularMotorFF.None,
    profileConstraints: AngularTrapezoidProfile.Constraints,
    drivePIDConstants: PIDConstants,
    velocityFF: AngularMotorFF,
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
            turnEncoder: PositionEncoder,
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
            turnEncoder: PositionEncoder,
            driveMotor: DM,
            turnPIDConstants: PIDConstants,
            turnFF: AngularMotorFF = AngularMotorFF.None,
            profileConstraints: AngularTrapezoidProfile.Constraints,
            drivePIDConstants: PIDConstants,
            velocityFF: AngularMotorFF,
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
        val drivePIDConstants: PIDConstants,
        val velocityFF: AngularMotorFF,
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
 * Represents a Profiled Swerve Module without motor configuration.
 *
 */
public open class NonConfigurableProfiledSwerveModule(
    turnMotor: EncoderMotorController,
    turnEncoder: PositionEncoder,
    driveMotor: EncoderMotorController,
    turnPIDConstants: PIDConstants,
    public val turnFF: AngularMotorFF = AngularMotorFF.None,
    public val profileConstraints: AngularTrapezoidProfile.Constraints,
    drivePIDConstants: PIDConstants,
    velocityFF: AngularMotorFF,
    turnPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    useOnboardPIDIfAvailable: Boolean = false,
): NonConfigurableSwerveModule(
    turnMotor, turnEncoder, driveMotor, turnPIDConstants, drivePIDConstants, velocityFF, turnPrecision, useOnboardPIDIfAvailable
) {
    protected override val turnMotorPositionSetter: (Angle) -> Unit = if(useOnboardPIDIfAvailable && turnMotor is FeedbackMotorController){

        // turnMotor smart casts to FeedbackMotorController here, which means that setAngularPosition is allowed.

        if (turnPrecision is Precision.Within){
            // return value
            {
                if(turnEncoder.angularPosition-it !in turnPrecision.allowableError) {
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
            getInput = {turnEncoder.angularPosition},
            target = Angle(0.0),
            constraints = profileConstraints,
            feedforward = turnFF,
            selfSustain = true,
            outputRange = -12.volts..12.volts
        )
        // Needed so that the return lambda isn't confused to be part of the superPIDController

        if(turnPrecision is Precision.Within){
            // return value
            {
                if(controller.target != it){controller.target = it}
                if (turnEncoder.angularPosition-it !in turnPrecision.allowableError){
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

}
