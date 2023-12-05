package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit

/*
NOTE TO SELF:
WPILib API is changing in 2024, so class will have to be changed accordingly:

fyi, the API is changing for 2024 so that calculate() takes initial position, unprofiled goal, and dt, and returns profiled setpoint for dt in the future. That way, you can reuse the instance.
https://github.com/wpilibsuite/allwpilib/pull/5457 
 */

/**
 * A wrapper for WPILib's [TrapezoidProfile] class, adding support for angular units.
 * For instance, the constraints are measured with max angular acceleration and max angular velocity.
 */
@JvmInline
public value class LinearTrapezoidProfile(
    public val siValue: TrapezoidProfile
){
}



