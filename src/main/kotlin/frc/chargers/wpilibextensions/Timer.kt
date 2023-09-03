package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.Timer

/**
 * A wrapper class for WPILib's Timer that uses Kmeasure's Time Unit instead.
 *
 * @see Timer
 */
public object Timer{
    public fun getFPGATimestamp(): Time = Timer.getFPGATimestamp().ofUnit(seconds)

    public val fpgaTimestamp: Time
        get() = getFPGATimestamp()
    public fun getMatchTime(): Time = Timer.getMatchTime().ofUnit(seconds)
    public fun delay(time: Time){
        Timer.delay(time.inUnit(seconds))
    }
}
