package frc.chargers.wpilibextensions.motorcontrol

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.chargers.utils.math.equations.stallTorqueToVoltage
import frc.chargers.utils.math.units.Torque

public var MotorController.speed: Double
    get() = this.get()
    set(value) { this.set(value) }

public fun MotorController.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}

public fun MotorController.setStallTorque(torque: Torque) {
    setVoltage(
        stallTorqueToVoltage(torque)
    )
}


