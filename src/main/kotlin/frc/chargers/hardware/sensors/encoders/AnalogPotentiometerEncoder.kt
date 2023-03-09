package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.Degrees
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogPotentiometer

public class AnalogPotentiometerPositionEncoder(input: AnalogInput, scale: Double = 1.0, offset: Angle = 0.degrees) :
    AnalogPotentiometer(input, scale, offset.inUnit(Degrees)), PositionEncoder {
    override val angularPosition: Angle
        get() = get().ofUnit(Degrees)
}