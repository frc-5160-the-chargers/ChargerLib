package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.Voltage
import edu.wpi.first.math.filter.SlewRateLimiter
import frc.chargers.utils.math.units.VoltageRate


/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [Voltage] input.
 */
public class VoltageRateLimiter{


    private val rateLimiter: SlewRateLimiter

    public constructor(rateLimit: VoltageRate){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
    }

    public constructor(
        positiveLimit: VoltageRate,
        negativeLimit: VoltageRate,
        initialValue: VoltageRate
    ){
        rateLimiter = SlewRateLimiter(
            positiveLimit.siValue,
            negativeLimit.siValue,
            initialValue.siValue
        )
    }


    public fun calculate(input: Voltage): Voltage =
        Voltage(rateLimiter.calculate(input.siValue))

    public fun reset(value: Voltage): Unit =
        rateLimiter.reset(value.siValue)
}