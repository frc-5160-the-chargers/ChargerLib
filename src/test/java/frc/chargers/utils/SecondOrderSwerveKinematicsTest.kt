package frc.chargers.utils

import edu.wpi.first.math.geometry.Translation2d
import org.junit.jupiter.api.Test

internal class SecondOrderSwerveKinematicsTest {
    @Test
    fun toSwerveModuleState() {
        val kinematics = SecondOrderSwerveKinematics(
            Translation2d(0.5,0.5),
            Translation2d(-0.5,0.5),
            Translation2d(0.5,-0.5),
            Translation2d(-0.5,-0.5)
        )
    }
}