package frc.chargers.constants

import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.units.inches

/**
 * A constants object for storing MK4i swerve module-related constants.
 */
public object MK4i {
    public const val TURN_GEAR_RATIO: Double = 7.0 / 150.0
    public const val GEAR_RATIO_L1: Double = 1.0 / 8.14
    public const val GEAR_RATIO_L2: Double = 1.0 / 6.75
    public const val GEAR_RATIO_L3: Double = 1.0 / 6.12

    public val WHEEL_DIAMETER: Length = 4.inches
}