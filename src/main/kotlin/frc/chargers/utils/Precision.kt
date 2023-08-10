package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Quantity
import frc.chargers.wpilibextensions.kinematics.StandardDeviation

/**
 * Example of use of Precision:
 *
 * when(precision) {
 *         Precision.AllowOvershoot -> {
 *             println("precision allows overshoot.")
 *         }
 *         is Precision.Within -> {
 *             println("Precision is within " + precision.allowableError)
 *         }
 *     }
 *
 * Note that Precision takes advantage of kotlin's smart casting
 * to cast the Precision to Precision.Within in the When statement
 */


public sealed class Precision<out D : AnyDimension> {
    public object AllowOvershoot : Precision<Nothing>()
    public class Within<D : AnyDimension>(public val allowableError: ClosedRange<Quantity<D>>) : Precision<D>() {
        public constructor(margin: Quantity<D>) : this(-margin..margin)

    }
}

public inline fun <D: AnyDimension> Precision<D>.processValue(
    whenValueExists: (Precision.Within<D>) -> Unit,
    whenAllowOvershoot: () -> Unit){
    if (this is Precision.Within<D>){
        whenValueExists(this)
    }else{
        whenAllowOvershoot()
    }
}









