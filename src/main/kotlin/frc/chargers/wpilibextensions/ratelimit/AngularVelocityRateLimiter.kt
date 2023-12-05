package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.AngularAcceleration
import com.batterystaple.kmeasure.quantities.AngularVelocity
import edu.wpi.first.math.filter.SlewRateLimiter



/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [AngularVelocity] input.
 */
public class AngularVelocityRateLimiter{


    private val rateLimiter: SlewRateLimiter

    public constructor(rateLimit: AngularAcceleration){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
    }

    public constructor(
        positiveLimit: AngularAcceleration,
        negativeLimit: AngularAcceleration,
        initialValue: AngularAcceleration
    ){
        require(positiveLimit.siValue > 0.0){"Positive Rate Limit must be a positive value."}
        require(negativeLimit.siValue < 0.0){"Negative Rate Limit must be a negative value."}

        rateLimiter = SlewRateLimiter(
            positiveLimit.siValue,
            negativeLimit.siValue,
            initialValue.siValue
        )
    }


    public fun calculate(input: AngularVelocity): AngularVelocity =
        AngularVelocity(rateLimiter.calculate(input.siValue))

    public fun reset(value: AngularVelocity): Unit =
        rateLimiter.reset(value.siValue)
}