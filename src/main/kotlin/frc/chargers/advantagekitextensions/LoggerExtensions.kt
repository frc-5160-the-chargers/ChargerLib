package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.wpilibextensions.fpgaTimestampReal
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger.*

internal var AK_LOGGABLE_REPLAY_TABLE: LogTable? = null
internal var AK_LOGGABLE_REAL_TABLE: LogTable? = null

/**
 * Records the output of a generic [Quantity].
 */
public fun <D: AnyDimension> recordOutput(key: String, value: Quantity<D>){
    recordOutput("$key(SI Value)", value.siValue)
}

/**
 * Records the output of a generic [AdvantageKitLoggable].
 */
public fun <T: AdvantageKitLoggable<T>> recordOutput(key: String, value: T){
    if (hasReplaySource()){
        AK_LOGGABLE_REPLAY_TABLE?.let { value.pushToLog(it, key) }
    }else{
        AK_LOGGABLE_REAL_TABLE?.let { value.pushToLog(it, key) }
    }
}

/**
 * Runs a code block while logging it's latency.
 */
public inline fun runAndLogLatency(logName: String, toRun: () -> Unit){
    val startTime = fpgaTimestampReal()
    toRun()
    recordOutput("$logName(ms)", (fpgaTimestampReal() - startTime).inUnit(milli.seconds))
}




