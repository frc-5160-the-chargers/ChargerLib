package frc.chargers.wpilibextensions.ratelimit

import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.abs
import edu.wpi.first.math.filter.SlewRateLimiter
import frc.chargers.utils.math.units.VoltageRate


/**
 * A wrapper over WPILib's [SlewRateLimiter] that rate-limits a [Voltage] input.
 */
public class VoltageRateLimiter{


    private val rateLimiter: SlewRateLimiter
    private val onlyLimitPositiveAccel: Boolean
    private var previousInput = Voltage(0.0)

    public constructor(rateLimit: VoltageRate, onlyLimitPositiveAccel: Boolean = false){
        rateLimiter = SlewRateLimiter(rateLimit.siValue)
        this.onlyLimitPositiveAccel = onlyLimitPositiveAccel
    }

    public constructor(
        positiveLimit: VoltageRate,
        negativeLimit: VoltageRate,
        initialValue: VoltageRate
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


    public fun calculate(input: Voltage): Voltage =
        if (onlyLimitPositiveAccel && abs(input) < abs(previousInput)){
            previousInput = input
            rateLimiter.reset(input.siValue)
            input
        }else{
            previousInput = input
            Quantity(rateLimiter.calculate(input.siValue))
        }

    public fun reset(value: Voltage): Unit =
        rateLimiter.reset(value.siValue)
}