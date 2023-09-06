package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.rotations
import edu.wpi.first.wpilibj.DigitalSource
import edu.wpi.first.wpilibj.DutyCycle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder

public class ChargerDutyCycleEncoder: DutyCycleEncoder, PositionEncoder{

    public constructor(channel: Int): super(channel)
    public constructor(source: DigitalSource): super(source)
    public constructor(dutyCycle: DutyCycle): super(dutyCycle)

    override val angularPosition: Angle
        get() = get().ofUnit(rotations)


}