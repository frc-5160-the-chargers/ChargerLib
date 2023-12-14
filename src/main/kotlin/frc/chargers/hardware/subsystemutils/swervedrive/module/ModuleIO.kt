package frc.chargers.hardware.subsystemutils.swervedrive.module

import com.batterystaple.kmeasure.quantities.*

public interface ModuleIO{
    public val logTab: String

    public val direction: Angle
    public val turnSpeed: AngularVelocity

    public val speed: AngularVelocity
    public val wheelTravel: Angle

    public var turnVoltage: Voltage
    public var driveVoltage: Voltage
}


