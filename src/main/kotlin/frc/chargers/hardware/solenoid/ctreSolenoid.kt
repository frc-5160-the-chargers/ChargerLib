package frc.chargers.hardware.solenoid

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid

public fun ctreSolenoid(
    channel: Int,
    moduleID: Int = 0
): Solenoid = Solenoid(
    moduleID,
    PneumaticsModuleType.CTREPCM,
    channel
)

public fun ctreDoubleSolenoid(
    forwardChannel: Int,
    reverseChannel: Int,
    moduleID: Int = 0
): DoubleSolenoid = DoubleSolenoid(
    moduleID,
    PneumaticsModuleType.CTREPCM,
    forwardChannel,
    reverseChannel,
)