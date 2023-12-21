package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.Voltage

/**
 * A generic interface representing a piece of hardware and/or a motor
 * which can return an applied voltage output.
 */
public interface VoltageProvider {
    public val appliedVoltage: Voltage
}