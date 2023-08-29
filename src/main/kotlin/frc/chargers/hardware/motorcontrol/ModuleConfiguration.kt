package frc.chargers.hardware.motorcontrol

public data class ModuleConfiguration<TMC: MotorConfiguration, DMC: MotorConfiguration>(
    var turnMotorConfig: TMC,
    var driveMotorConfig: DMC
): MotorConfiguration {
    public inline fun turnMotor(configure: TMC.() -> Unit){
        turnMotorConfig = turnMotorConfig.apply(configure)
    }

    public inline fun driveMotor(configure: DMC.() -> Unit){
        driveMotorConfig = driveMotorConfig.apply(configure)
    }

}