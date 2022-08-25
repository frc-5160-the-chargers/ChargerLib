package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.Degrees
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
        get() = -navX.angle.ofUnit(Degrees)

    public val isConnected: Boolean
        get() = navX.isConnected
}