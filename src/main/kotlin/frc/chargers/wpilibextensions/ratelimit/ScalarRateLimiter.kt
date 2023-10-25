package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.Frequency
import com.batterystaple.kmeasure.quantities.Scalar
import edu.wpi.first.math.filter.SlewRateLimiter


/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [Scalar].
 */
public class ScalarRateLimiter{


    private val rateLimiter: SlewRateLimiter

    public constructor(rateLimit: Frequency){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
    }

    public constructor(
        positiveLimit: Frequency,
        negativeLimit: Frequency,
        initialValue: Frequency
    ){
        rateLimiter = SlewRateLimiter(
            positiveLimit.siValue,
            negativeLimit.siValue,
            initialValue.siValue
        )
    }

    public fun calculate(input: Double): Double =
        rateLimiter.calculate(input)

    public fun reset(value: Double): Unit =
        rateLimiter.reset(value)


    public fun calculate(input: Scalar): Scalar =
        Scalar(rateLimiter.calculate(input.siValue))

    public fun reset(value: Scalar): Unit =
        rateLimiter.reset(value.siValue)


}