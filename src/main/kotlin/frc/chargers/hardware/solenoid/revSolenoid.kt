package frc.chargers.hardware.solenoid

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid

public fun revSolenoid(
    channel: Int,
    moduleID: Int = 1
): Solenoid = Solenoid(
    moduleID,
    PneumaticsModuleType.REVPH,
    channel
)

public fun revDoubleSolenoid(
    forwardChannel: Int,
    reverseChannel: Int,
    moduleID: Int = 1
): DoubleSolenoid = DoubleSolenoid(
    moduleID,
    PneumaticsModuleType.REVPH,
    forwardChannel,
    reverseChannel,
)