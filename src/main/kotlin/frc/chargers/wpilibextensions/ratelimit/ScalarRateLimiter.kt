package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.Frequency
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.abs
import edu.wpi.first.math.filter.SlewRateLimiter
import kotlin.math.abs


/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [Scalar].
 */
public class ScalarRateLimiter{


    private val rateLimiter: SlewRateLimiter
    private val onlyLimitPositiveAccel: Boolean
    private var previousInput: Scalar = Scalar(0.0)

    public constructor(rateLimit: Frequency, onlyLimitPositiveAccel: Boolean = false) {
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
        this.onlyLimitPositiveAccel = onlyLimitPositiveAccel
    }

    public constructor(
        positiveLimit: Frequency,
        negativeLimit: Frequency,
        initialValue: Frequency
    ){
        require(positiveLimit.siValue > 0.0){"Positive Rate Limit must be a positive value."}
        require(negativeLimit.siValue < 0.0){"Negative Rate Limit must be a negative value."}
        this.onlyLimitPositiveAccel = false
        rateLimiter = SlewRateLimiter(
            positiveLimit.siValue,
            negativeLimit.siValue,
            initialValue.siValue
        )
    }

    public fun calculate(input: Double): Double =
        if (onlyLimitPositiveAccel && abs(input) < abs(previousInput.siValue)){
            previousInput = Scalar(input)
            rateLimiter.reset(input)
            input
        }else{
            previousInput = Scalar(input)
            rateLimiter.calculate(input)
        }


    public fun reset(value: Double): Unit =
        rateLimiter.reset(value)


    public fun calculate(input: Scalar): Scalar =
        if (onlyLimitPositiveAccel && abs(input) < abs(previousInput)){
            previousInput = input
            rateLimiter.reset(input.siValue)
            input
        }else{
            previousInput = input
            Quantity(rateLimiter.calculate(input.siValue))
        }

    public fun reset(value: Scalar): Unit =
        rateLimiter.reset(value.siValue)


}