package frc.chargers.constants

import frc.chargers.controls.pid.PIDConstants

/**
 * An interface denoting an object that provides PID constants for turning.
 */
public interface TurnPIDConstants {
    public val turnPIDConstants: PIDConstants

    public companion object {
        public operator fun invoke(kP: Double, kI: Double, kD: Double): TurnPIDConstants =
            TurnPIDConstantsImpl(PIDConstants(kP = kP, kI = kI, kD = kD))
        public operator fun invoke(turnPIDConstants: PIDConstants): TurnPIDConstants =
            TurnPIDConstantsImpl(turnPIDConstants)
    }
}

private data class TurnPIDConstantsImpl(override val turnPIDConstants: PIDConstants) : TurnPIDConstants