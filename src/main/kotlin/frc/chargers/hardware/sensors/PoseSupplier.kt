package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Distance
import frc.chargers.utils.Measurement
import frc.chargers.utils.NullableMeasurement
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.StandardDeviation

/**
 * Represents a generic device that can supply the current pose of the robot it's in.
 * This can encompass drivetrains and smart vision sensors(like the limelight).
 *
 * RobotPoseSupplier always uses [WPILib's field coordinate system](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html) for measurement.
 */
public interface RobotPoseSupplier {

    public val poseStandardDeviation: StandardDeviation

    public val robotPoseMeasurement: NullableMeasurement<UnitPose2d>

    public val robotPose: UnitPose2d?
        get() = robotPoseMeasurement.nullableValue

}

/**
 * Represents a camera/sensor that can obtain its pose through various methods.
 * Note that this is distinct from a RobotPoseSupplier.
 *
 * CameraPoseSupplier always uses [WPILib's field coordinate system](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html) for measurement.
 */
public interface CameraPoseSupplier {
    public val cameraPoseMeasurement: Measurement<UnitPose2d>

    public val poseStandardDeviation: StandardDeviation

    public val cameraPose: UnitPose2d
        get() = cameraPoseMeasurement.value

    public fun asRobotPoseSupplier(xOffset: Distance, yOffset: Distance): RobotPoseSupplier =
        object: RobotPoseSupplier{
            override val robotPoseMeasurement: Measurement<UnitPose2d>
                get(){
                    val initial = cameraPoseMeasurement
                    val initialPose = initial.value
                    return Measurement(
                        UnitPose2d(initialPose.x-xOffset, initialPose.y-yOffset, initialPose.rotation),
                        initial.latency
                    )
                }

            override val poseStandardDeviation = this@CameraPoseSupplier.poseStandardDeviation
        }
}



