package frc.chargers.hardware.subsystems.drivetrain
/*

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import io.mockk.mockk
import io.mockk.verify
import kotlin.test.BeforeTest
import kotlin.test.Test

internal class BasicDifferentialDrivetrainTest {

@BeforeTest
fun setup() {
assert(HAL.initialize(500, 0))
}

@Test
fun `Tank drive with equal power should drive wheels equally`() {
val testPowers = listOf(0.0, 1.0, 0.2, 0.8, -1.0, -0.2, -0.8)

val leftMotorsMock = mockk<MotorControllerGroup>()
val rightMotorsMock = mockk<MotorControllerGroup>()

val drivetrain = BasicDifferentialDrivetrain(
leftMotors = leftMotorsMock,
rightMotors = rightMotorsMock
)

for (leftPower in testPowers) {
for (rightPower in testPowers) {
drivetrain.tankDrive(
leftPower = leftPower,
rightPower = rightPower
)
}
}

verify {
for (leftPower in testPowers) {
for (rightPower in testPowers) {
leftMotorsMock.set(leftPower)
rightMotorsMock.set(rightPower)
}
}
}
}

@Test
fun arcadeDrive() {
}

@Test
fun curvatureDrive() {
}

@Test
fun stop() {
}
}
*/