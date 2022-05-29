package frc.robot.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M : MotorController> MotorControllerGroup(motorController: M, vararg motorControllers: M, configure: (motorController: M) -> Unit): MotorControllerGroup {
    configure(motorController)
    motorControllers.forEach { otherMotorController -> configure(otherMotorController) }

    return MotorControllerGroup(motorController, *motorControllers)
}

/**
 * A convenience function for creating and configuring a [MotorControllerGroup]
 */
public fun <M : MotorController> MotorControllerGroup(motorControllers: Array<M>, configure: (motorController: M) -> Unit): MotorControllerGroup {
    motorControllers.forEach { motorController -> configure(motorController) }

    return MotorControllerGroup(motorControllers)
}