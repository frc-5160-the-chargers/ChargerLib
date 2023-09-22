package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3

/**
 * Represents a standard deviation
 */
public sealed class StandardDeviation {
    public data object Default: StandardDeviation()

    public class Of(
        public val x: Distance,
        public val y: Distance,
        public val angle: Angle
    ): StandardDeviation(){
        public fun getVector(): Matrix<N3, N1> = VecBuilder.fill(
            x.siValue,
            y.siValue,
            angle.siValue
        )
    }
}

public inline fun StandardDeviation.processValue(
    whenValueExists: (StandardDeviation.Of) -> Unit,
    whenDefault: () -> Unit
){
    if (this is StandardDeviation.Of){
        whenValueExists(this)
    }else{
        whenDefault()
    }
}