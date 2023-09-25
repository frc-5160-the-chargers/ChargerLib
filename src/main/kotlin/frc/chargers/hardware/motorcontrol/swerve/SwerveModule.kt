package frc.chargers.hardware.motorcontrol.swerve

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity

public class SwerveModule(
    getDirection: () -> Angle,
    setDirection: (Angle) -> Unit,
    getVelocity: () -> AngularVelocity,
    setVelocity: (AngularVelocity) -> Unit
) {

}