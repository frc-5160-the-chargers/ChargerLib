package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Length

/**
 * A provider that provides generic gearRatio/wheelDiameter data.
 */
public interface WheelRatioProvider {
    public val gearRatio: Double
    public val wheelDiameter: Length
}