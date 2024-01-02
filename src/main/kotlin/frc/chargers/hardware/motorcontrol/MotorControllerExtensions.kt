package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M, C : HardwareConfiguration> MotorControllerGroup(motorController: M, vararg motorControllers: M, configuration: C): MotorControllerGroup
        where M: MotorController, M: HardwareConfigurable<C> {
    motorController.configure(configuration)
    motorControllers.forEach { it.configure(configuration) }

    return MotorControllerGroup(motorController, *motorControllers)
}

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M, C: HardwareConfiguration> MotorControllerGroup(motorControllers: Array<M>, configuration: C): MotorControllerGroup
        where M: MotorController, M: HardwareConfigurable<C> {
    motorControllers.forEach { it.configure(configuration) }

    return MotorControllerGroup(motorControllers)
}

/**
 * An extension property that acts as a replacement for MotorController.get()
 * and MotorController.set().
 */
public var MotorController.speed: Double
    get() = this.get()
    set(value) { this.set(value) }

/**
 * Sets the voltage of a [MotorController] using the kmeasure voltage unit.
 */
public fun MotorController.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}



