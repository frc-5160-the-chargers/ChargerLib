package frc.chargers.wpilibextensions.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.chargers.utils.math.equations.stallTorqueToVoltage
import frc.chargers.utils.math.units.Torque

public var MotorController.speed: Double
    get() = this.get()
    set(value) { this.set(value) }

public var MotorController.voltage: Voltage
    get() = RobotController.getInputVoltage().ofUnit(volts) * speed
    set(value){
        setVoltage(value)
    }

@JvmName("setVoltageFunction")
public fun MotorController.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}

public fun MotorController.setStallTorque(torque: Torque) {
    setVoltage(
        stallTorqueToVoltage(torque)
    )
}


