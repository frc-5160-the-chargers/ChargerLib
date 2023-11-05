package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import frc.chargers.constants.AndymarkKitbot
import frc.chargers.hardware.subsystems.drivetrain.DEFAULT_GEAR_RATIO

public data class TankDriveConstants(
    val invertMotors: Boolean = false,
    val gearRatio: Double = DEFAULT_GEAR_RATIO,
    val wheelDiameter: Length,
    val width: Distance,
){
    public companion object{
        public fun andymark(invertMotors: Boolean = false): TankDriveConstants =
            TankDriveConstants(
                invertMotors,
                AndymarkKitbot.GEAR_RATIO,
                AndymarkKitbot.WHEEL_DIAMETER,
                AndymarkKitbot.WIDTH
            )
    }
}
