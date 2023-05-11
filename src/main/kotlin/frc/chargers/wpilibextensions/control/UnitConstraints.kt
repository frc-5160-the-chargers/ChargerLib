package frc.chargers.wpilibextensions.control

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.quantities.ofUnit
import edu.wpi.first.math.trajectory.TrapezoidProfile



public class UnitConstraints(
    public val maxVelocity: Velocity,
    public val maxAcceleration: Acceleration
){
    public val hi = TrapezoidProfile.Constraints(5.0,5.0)
    public fun asProfile(): TrapezoidProfile{
        return TrapezoidProfile.Constraints(maxVelocity.ofUnit(meters/seconds),maxAcceleration.ofUnit(meters/(seconds*seconds)))
    }
}
