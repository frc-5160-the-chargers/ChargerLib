package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.units.inches

/**
 * A class used to hold constants for an [frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain].
 */
public data class DiffDriveConstants(
    val invertMotors: Boolean = false,
    val gearRatio: Double = DEFAULT_GEAR_RATIO,
    val wheelDiameter: Length,
    val width: Distance,
){
    public companion object{
        public fun andyMark(invertMotors: Boolean = false): DiffDriveConstants =
            DiffDriveConstants(
                invertMotors,
                1.0 / 10.71,
                6.inches,
                27.inches
            )
    }
}
