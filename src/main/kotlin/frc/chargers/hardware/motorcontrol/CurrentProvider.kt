package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.Current


/**
 * A generic interface representing a piece of hardware and/or a motor
 * which can return its own temperature.
 */
public interface CurrentProvider {
    public val appliedCurrent: Current
}