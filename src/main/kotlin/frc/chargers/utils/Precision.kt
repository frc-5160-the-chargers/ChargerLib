package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

/**
 * A class that holds data about the Precision of a certain mechanism, or generic item
 */
public sealed class Precision<out D : AnyDimension> {
    public data object AllowOvershoot : Precision<Nothing>()
    public class Within<D : AnyDimension>(public val allowableError: ClosedRange<Quantity<D>>) : Precision<D>() {
        public constructor(margin: Quantity<D>) : this(-margin..margin)
    }
}

public fun <D: AnyDimension> Quantity<D>.within(precision: Precision<D>): Boolean =
    if (precision is Precision.Within){
        this in precision.allowableError
    }else{
        false
    }










