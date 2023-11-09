package frc.chargers.hardware.inputdevices

import frc.chargers.wpilibextensions.Alert

/**
 * A utility function that warns the user when they instantiate a hardware device in sim.
 *
 * So far, per-device simulation is not supported in ChargerLib;
 * it is recommended to use AdvantageKit's IO layers, with WPILib Physics simulation
 * instead of vendor-specific simulation.
 */
public fun warnIfInSimulation(deviceName: String){
    val text = "WARNING: a device with name '$deviceName' was instantiated in SIM. " +
            "\n So far, this device does NOT support simulation; accessing it will likely cause errors."
    val alert = Alert.warning("DeviceAlerts", text)

    alert.active = true
    println(text)
}