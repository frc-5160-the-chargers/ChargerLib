package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.wpilibextensions.fpgaTimestamp
import org.littletonrobotics.junction.Logger.*

public fun <D: AnyDimension> recordOutput(key: String, input: Quantity<D>){
    recordOutput("$key(SI Value)", input.siValue)
}

public inline fun runAndLogLatency(logName: String, toRun: () -> Unit){
    val startTime = fpgaTimestamp()
    toRun()
    recordOutput("$logName(ms)", (fpgaTimestamp() - startTime).inUnit(milli.seconds))
}


