package frc.chargers.wpilibextensions.motorcontrol

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup

public var MotorController.speed: Double
    get() = this.get()
    set(value) { this.set(value) }

public fun MotorController.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}

public var MotorControllerGroup.speed: Double
    get() = this.get()
    set(value) { this.set(value) }

public fun MotorControllerGroup.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}

