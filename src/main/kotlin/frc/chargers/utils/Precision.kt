package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

/**
 * A class that holds data about the Precision of a certain mechanism or class.
 */
public sealed class Precision<out D : AnyDimension> {
    public data object AllowOvershoot : Precision<Nothing>()
    public class Within<D : AnyDimension>(public val allowableError: ClosedRange<Quantity<D>>) : Precision<D>() {
        public constructor(margin: Quantity<D>) : this(-margin..margin)
    }
}

/**
 * Determines whether a quantity is within a specified [Precision].
 *
 * If the precision is a [Precision.Within], it will return true if the value is between the allowable error or not.
 * Otherwise, it will return false.
 */
public fun <D: AnyDimension> Quantity<D>.within(precision: Precision<D>): Boolean =
    if (precision is Precision.Within){
        this in precision.allowableError
    }else{
        false
    }










