package frc.chargers.hardware.subsystems.differentialdrive.lowlevel

import com.batterystaple.kmeasure.quantities.*


/**
 * An interface representing the low level component of a
 * [frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain].
 *
 * There are 2 implementations of this class: one for the real robot ([DiffDriveIOReal])
 * and one for the sim robot ([DiffDriveIOSim]).
 */
public interface DiffDriveIO {
    public fun setVoltages(left: Voltage, right: Voltage)

    public val leftWheelTravel: Angle
    public val rightWheelTravel: Angle

    public val leftVelocity: AngularVelocity
    public val rightVelocity: AngularVelocity

    public val leftVoltage: Voltage
    public val rightVoltage: Voltage

    public var inverted: Boolean
}