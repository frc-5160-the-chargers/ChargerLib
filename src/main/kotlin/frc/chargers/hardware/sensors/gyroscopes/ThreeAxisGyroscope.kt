package frc.chargers.hardware.sensors.gyroscopes

import com.batterystaple.kmeasure.quantities.Angle
import edu.wpi.first.math.geometry.Rotation2d

public interface ThreeAxisGyroscope {
    public val yaw: Angle
    public val pitch: Angle
    public val roll: Angle
    public val rotation2d: Rotation2d
}