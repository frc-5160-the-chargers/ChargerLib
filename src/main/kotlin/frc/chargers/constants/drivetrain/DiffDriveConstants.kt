package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import frc.chargers.constants.AndymarkKitbot

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
        public fun andymark(invertMotors: Boolean = false): DiffDriveConstants =
            DiffDriveConstants(
                invertMotors,
                AndymarkKitbot.GEAR_RATIO,
                AndymarkKitbot.WHEEL_DIAMETER,
                AndymarkKitbot.WIDTH
            )
    }
}
