package frc.chargers.hardware.sensors

import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

/**
 * Represents a Robot pose Estimator, which always produces valid pose results.
 *
 * This class should not be implemented by vision cameras or sensors, as their pose results
 * are not always accurate, and thus the nullable parameters are necessary.
 */
public interface RobotPoseEstimator{
    public val robotPose: UnitPose2d
}

