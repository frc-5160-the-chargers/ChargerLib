package frc.chargers.wpilibextensions.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.chargers.utils.math.equations.stallTorqueToVoltage
import frc.chargers.utils.math.units.Torque

public var MotorController.speed: Double
    get() = this.get()
    set(value) { this.set(value) }


public var MotorController.voltage: Voltage
    @JvmName("busVoltage")
    // workaround implementation as supply/bus voltage is not included
    // within the MotorController interface.
    get() = when (this) {
        is TalonFX -> {
            supplyVoltage.value.ofUnit(volts) * speed
        }

        is CANSparkMax -> {
            busVoltage.ofUnit(volts) * speed
        }

        else -> {
            RobotController.getBatteryVoltage().ofUnit(volts) * speed
        }
    }
    @JvmName("busVoltage")
    set(value){
        setVoltage(value)
    }


public fun MotorController.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}

public fun MotorController.setStallTorque(torque: Torque) {
    setVoltage(
        stallTorqueToVoltage(torque)
    )
}


