package frc.chargers.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M,C : HardwareConfiguration> MotorControllerGroup(motorController: M, vararg motorControllers: M, configuration: C): MotorControllerGroup
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

