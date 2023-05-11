package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds



public interface AngularFeedForward {
    public fun calculate(
        currentAngle: Angle,
        velocityTarget: AngularVelocity,
        accelerationTarget: AngularAcceleration): Voltage

    public fun calculate(
        currentAngle: Angle,
        velocityTarget: AngularVelocity): Voltage
}

public interface LinearFeedForward {
    public fun calculate(
        velocityTarget: Velocity,
        accelerationTarget: Acceleration): Voltage

    public fun calculate(velocityTarget: Velocity): Voltage
}