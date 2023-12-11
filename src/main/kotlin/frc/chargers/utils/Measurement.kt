package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import org.littletonrobotics.junction.LogTable


/**
 * A [Double] accompanied by a timestamped.
 */
public data class DoubleMeasurement(
    val value: Double,
    val timestamp: Time,
): AdvantageKitLoggable<DoubleMeasurement>{
    override fun pushToLog(table: LogTable, category: String) {
        table.put("$category/value", value)
        table.put("$category/timestampSecs",timestamp.inUnit(seconds))
    }

    override fun getFromLog(table: LogTable, category: String): DoubleMeasurement =
        DoubleMeasurement(
            table.get("$category/value", 0.0),
            timestamp = table.get("$category/timestampSecs",0.0).ofUnit(seconds)
        )
}

/**
 * A [Quantity] accompanied by a timestamped.
 */
public data class QuantityMeasurement<D: AnyDimension>(
    val value: Quantity<D>,
    val timestamp: Time,
): AdvantageKitLoggable<QuantityMeasurement<D>>{
    override fun pushToLog(table: LogTable, category: String) {
        table.put("$category/value(SI unit)", value.siValue)
        table.put("$category/timestampSecs",timestamp.inUnit(seconds))
    }

    override fun getFromLog(table: LogTable, category: String): QuantityMeasurement<D> =
        QuantityMeasurement(
            Quantity(table.get("$category/value(SI unit)", 0.0)),
            timestamp = table.get("$category/timestampSecs",0.0).ofUnit(seconds)
        )
}


/**
 * Represents a Loggable value accompanied by a timestamp.
 */
public data class Measurement<T: AdvantageKitLoggable<T>>(
    @JvmField val value: T,
    val timestamp: Time
): AdvantageKitLoggable<Measurement<T>> {
    override fun pushToLog(table: LogTable, category: String) {
        value.pushToLog(table, "$category/value")
        table.put("$category/timestampSecs",timestamp.inUnit(seconds))
    }
    override fun getFromLog(table: LogTable, category: String): Measurement<T> =
        Measurement(
            value.getFromLog(table,"$category/value"),
            timestamp = table.get("$category/timestampSecs",0.0).ofUnit(seconds)
        )
}
