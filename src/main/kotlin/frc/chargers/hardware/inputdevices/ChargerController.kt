package frc.chargers.hardware.inputdevices

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.Button as WPIlibCommandButton
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs

public open class ChargerController(
    port: Int,
    private val deadband: Double = 0.0,
    private val defaultAxisThreshold: Double = 0.5
) : XboxController(port) {
    protected fun button(button: Button): Trigger =
        JoystickButton(this, button.value)
    protected fun button(axis: Axis, threshold: Double = defaultAxisThreshold): Trigger =
        JoystickAnalogButton(this, axis.value, threshold)

    protected fun Double.withDeadband(): Double =
        if (abs(this) <= deadband) { 0.0 } else { this }
}
