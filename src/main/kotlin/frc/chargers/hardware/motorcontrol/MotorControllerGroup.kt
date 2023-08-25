package frc.chargers.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M,C : MotorConfiguration> MotorControllerGroup(motorController: M, vararg motorControllers: M, configuration: C): MotorControllerGroup
    where M: MotorController, M: MotorConfigurable<C>{
    motorController.configure(configuration)
    motorControllers.forEach { it.configure(configuration) }

    return MotorControllerGroup(motorController, *motorControllers)
}

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M, C: MotorConfiguration> MotorControllerGroup(motorControllers: Array<M>, configuration: C): MotorControllerGroup
    where M: MotorController, M: MotorConfigurable<C>{
    motorControllers.forEach { it.configure(configuration) }

    return MotorControllerGroup(motorControllers)
}

