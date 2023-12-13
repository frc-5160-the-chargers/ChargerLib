package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.abs
import edu.wpi.first.math.filter.SlewRateLimiter



/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [Velocity] input.
 */
public class VelocityRateLimiter{


    private val rateLimiter: SlewRateLimiter
    private val onlyLimitPositiveAccel: Boolean
    private var previousInput = Velocity(0.0)

    public constructor(rateLimit: Acceleration, onlyLimitPositiveAccel: Boolean = false){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
        this.onlyLimitPositiveAccel = onlyLimitPositiveAccel
    }

    public constructor(
        positiveLimit: Acceleration,
        negativeLimit: Acceleration,
        initialValue: Acceleration
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


    public fun calculate(input: Velocity): Velocity =
        if (onlyLimitPositiveAccel && abs(input) < abs(previousInput)){
            previousInput = input
            rateLimiter.reset(input.siValue)
            input
        }else{
            previousInput = input
            Quantity(rateLimiter.calculate(input.siValue))
        }

    public fun reset(value: Velocity): Unit =
        rateLimiter.reset(value.siValue)
}