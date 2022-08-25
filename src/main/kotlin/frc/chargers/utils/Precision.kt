package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

public sealed class Precision<out D : AnyDimension> {
    public  object AllowOvershoot : Precision<Nothing>()
    public class Within<D : AnyDimension>(public val allowableError: ClosedRange<Quantity<D>>) : Precision<D>() {
        public constructor(margin: Quantity<D>) : this(-margin..margin)
    }
}