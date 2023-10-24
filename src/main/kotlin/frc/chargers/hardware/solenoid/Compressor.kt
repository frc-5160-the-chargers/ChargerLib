package frc.chargers.hardware.solenoid

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.PneumaticsModuleType

public fun ctreCompressor(moduleID: Int = 0): Compressor = Compressor(
    moduleID,
    PneumaticsModuleType.CTREPCM
)

public fun revCompressor(moduleID: Int = 1): Compressor = Compressor(
    moduleID,
    PneumaticsModuleType.REVPH
)