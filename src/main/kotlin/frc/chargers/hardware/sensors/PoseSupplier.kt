package frc.chargers.hardware.sensors

import frc.chargers.utils.Measurement
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

    public val robotPoseMeasurement: Measurement<UnitPose2d>?

    public val robotPose: UnitPose2d?
        get() = robotPoseMeasurement?.value
}



