package frc.robot.hardware.sensors

import com.batterystaple.kmeasure.Angle
import com.batterystaple.kmeasure.Degrees
import com.batterystaple.kmeasure.ofUnit
import com.kauailabs.navx.frc.AHRS

public class NavX : HeadingProvider {
    private val navX = AHRS()

    init {
        reset()
    }

    public fun reset() {
        navX.reset()
    }

    public override val heading: Angle
        get() = navX.angle.ofUnit(Degrees)

    public val isConnected: Boolean
        get() = navX.isConnected
}