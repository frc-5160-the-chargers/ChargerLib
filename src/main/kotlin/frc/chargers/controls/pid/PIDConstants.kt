package frc.chargers.controls.pid

import edu.wpi.first.math.controller.PIDController

/**
 * A data class representing the various constants needed to configure a PID controller.
 */
public data class PIDConstants(
    /**
     * The constant that weights the proportional PID term.
     */
    @JvmField
    public var kP: Double,

    /**
     * The constant that weights the integral PID term.
     */
    @JvmField
    public var kI: Double,

    /**
     * The constant that weights the derivative PID term.
     */
    @JvmField
    public var kD: Double
){
    override fun equals(other: Any?): Boolean = if (other is PIDConstants){
        (other.kP == this.kP && other.kI == this.kI && other.kD == this.kD)
    }else{
        false
    }

    override fun hashCode(): Int {
        var result = kP.hashCode()
        result = 31 * result + kI.hashCode()
        result = 31 * result + kD.hashCode()
        return result
    }
}

/**
 * Gets the [PIDConstants] of an existing [PIDController]
 */
public var PIDController.constants: PIDConstants
    get() = PIDConstants(p, i, d)
    set(newConstants) = setPID(newConstants.kP, newConstants.kI, newConstants.kD)