package frc.chargers.utils

import com.batterystaple.kmeasure.*

public sealed class Precision<out D : Dimension> {
    public  object AllowOvershoot : Precision<Nothing>()
    public class Within<D : Dimension>(public val allowableError: ClosedRange<DimensionedQuantity<D>>) : Precision<D>() {
        public constructor(margin: DimensionedQuantity<D>) : this(-margin..margin)
    }
}