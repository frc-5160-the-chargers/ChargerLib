package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.AngularAcceleration
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.abs
import edu.wpi.first.math.filter.SlewRateLimiter



/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [AngularVelocity] input.
 */
public class AngularVelocityRateLimiter{

    private val rateLimiter: SlewRateLimiter
    private val onlyLimitPositiveAccel: Boolean
    private var previousInput: AngularVelocity = AngularVelocity(0.0)

    public constructor(rateLimit: AngularAcceleration, onlyLimitPositiveAccel: Boolean){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
        this.onlyLimitPositiveAccel = onlyLimitPositiveAccel
    }

    public constructor(
        positiveLimit: AngularAcceleration,
        negativeLimit: AngularAcceleration,
        initialValue: AngularAcceleration
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


    public fun calculate(input: AngularVelocity): AngularVelocity =
        if (onlyLimitPositiveAccel && abs(input) < abs(previousInput)){
            previousInput = input
            rateLimiter.reset(input.siValue)
            input
        }else{
            previousInput = input
            Quantity(rateLimiter.calculate(input.siValue))
        }

    public fun reset(value: AngularVelocity): Unit =
        rateLimiter.reset(value.siValue)
}