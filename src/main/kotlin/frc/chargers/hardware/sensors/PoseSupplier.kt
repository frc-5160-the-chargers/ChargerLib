package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Time
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.StandardDeviation

public interface RobotPoseSupplier {

    public val poseStandardDeviation: StandardDeviation

    public val robotPoseMeasurement: Measurement<UnitPose2d>

    public val robotPose: UnitPose2d
        get() = robotPoseMeasurement.value

}

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
                        initial.latency,
                        initial.isValid
                    )
                }

            override val poseStandardDeviation = this@CameraPoseSupplier.poseStandardDeviation
        }
}



