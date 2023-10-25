package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Velocity
import edu.wpi.first.math.filter.SlewRateLimiter



/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [Velocity] input.
 */
public class VelocityRateLimiter{


    private val rateLimiter: SlewRateLimiter

    public constructor(rateLimit: Acceleration){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
    }

    public constructor(
        positiveLimit: Acceleration,
        negativeLimit: Acceleration,
        initialValue: Acceleration
    ){
        rateLimiter = SlewRateLimiter(
            positiveLimit.siValue,
            negativeLimit.siValue,
            initialValue.siValue
        )
    }


    public fun calculate(input: Velocity): Velocity =
        Velocity(rateLimiter.calculate(input.siValue))

    public fun reset(value: Velocity): Unit =
        rateLimiter.reset(value.siValue)
}