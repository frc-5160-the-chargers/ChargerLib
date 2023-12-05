package frc.chargers.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.math.MathUtil
import kotlin.math.abs

/**
 * A convenience object to store the angles for the POV of an xbox controller.
 *
 * @see ChargerController
 */
public object POV{
    public val up: Angle = 0.degrees
    public val upRight: Angle = 45.degrees
    public val right: Angle = 90.degrees
    public val downRight: Angle = 135.degrees
    public val down: Angle = 180.degrees
    public val downLeft: Angle = 225.degrees
    public val left: Angle = 270.degrees
    public val upLeft: Angle = 315.degrees
}

public fun Double.withDeadbandOf(controller: ChargerController): Double =
    if (abs(this) <= controller.deadband) { 0.0 } else { this }

public fun Double.withScaledDeadbandOf(controller: ChargerController): Double =
    MathUtil.applyDeadband(this, controller.deadband, 1.0)

