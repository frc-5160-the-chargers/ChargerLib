package frc.chargers.hardware.sensors.vision

import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d

/*
    The following functions are used to provide dummy vision data
    To be logged,
    when the data is marked invalid.

    Since data must be logged every loop, these values are fallen back to when nessecary.
 */

public fun emptyAprilTagVisionData(): VisionData<VisionResult.AprilTag> =
    VisionData(
        timestamp = fpgaTimestamp(),
        bestTarget = VisionResult.AprilTag(
            0.0,0.0,0.0,0, UnitTransform3d()
        )
    )

public fun emptyMLVisionData(): VisionData<VisionResult.ML> =
    VisionData(
        timestamp = fpgaTimestamp(),
        bestTarget = VisionResult.ML(
            0.0,0.0,0.0,0
        )
    )

public fun emptyGenericVisionData(): VisionData<VisionResult.Generic> =
    VisionData(
        timestamp = fpgaTimestamp(),
        bestTarget = VisionResult.Generic(0.0,0.0,0.0)
    )