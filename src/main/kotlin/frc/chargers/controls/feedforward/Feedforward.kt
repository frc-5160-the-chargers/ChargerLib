package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.dimensions.VelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward

/**
 * Represents a Generic Feedforward.
 * A feedforward is an equation which estimates the Control Effort(type E) required
 * to achieve a certain output(type O).
 *
 *
 * See [here](https://www.controleng.com/articles/feed-forwards-augment-pid-control/) for an explanation of feedforward.
 */
public fun interface Feedforward<I: AnyDimension, O: AnyDimension> {
    public fun calculate(value: Quantity<I>): Quantity<O>
}


/**
 * Wraps WPILib's [SimpleMotorFeedforward], adding units support.
 *
 * This function takes in an [AngularVelocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: AngularMotorFFConstants,
    crossinline getTargetAccel: () -> AngularAcceleration = { AngularAcceleration(0.0) }
): Feedforward<AngularVelocityDimension, VoltageDimension>{
    val baseFF = SimpleMotorFeedforward(constants.kS.siValue, constants.kV.siValue, constants.kA.siValue)
    return Feedforward{ Voltage(baseFF.calculate(it.siValue, getTargetAccel().siValue)) }
}

/**
 * Wraps WPILib's [SimpleMotorFeedforward], adding units support.
 *
 * This function takes in a [Velocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: ArmFFConstants,
    crossinline getAngle: () -> Angle,
    crossinline getTargetAccel: () -> AngularAcceleration = { AngularAcceleration(0.0) }
): Feedforward<AngularVelocityDimension, VoltageDimension>{
    val baseFF = ArmFeedforward(constants.kS.siValue, constants.kG.siValue, constants.kV.siValue, constants.kA.siValue)

    return Feedforward{
        Voltage(
            baseFF.calculate(getAngle().siValue, it.siValue, getTargetAccel().siValue)
        )
    }
}

/**
 * Wraps WPILib's [ArmFeedforward], adding units support.
 *
 * This function takes in an [AngularVelocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: LinearMotorFFConstants,
    crossinline getTargetAccel: () -> Acceleration = { Acceleration(0.0) }
): Feedforward<VelocityDimension, VoltageDimension>{
    val baseFF = SimpleMotorFeedforward(constants.kS.siValue, constants.kV.siValue, constants.kA.siValue)
    return Feedforward{ Voltage(baseFF.calculate(it.siValue, getTargetAccel().siValue)) }
}

/**
 * Wraps WPILib's [ElevatorFeedforward], adding units support.
 *
 * This function takes in a [Velocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: ElevatorFFConstants,
    crossinline getTargetAccel: () -> AngularAcceleration = { AngularAcceleration(0.0) }
): Feedforward<VelocityDimension, VoltageDimension>{
    val baseFF = ElevatorFeedforward(constants.kS.siValue, constants.kG.siValue, constants.kV.siValue, constants.kA.siValue)

    return Feedforward{
        Voltage(
            baseFF.calculate(it.siValue, getTargetAccel().siValue)
        )
    }
}



