package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.geometry.Rotation2d
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds

private fun convertToAngle(sineValue: Double,cosineValue: Double): Double = if(sineValue > 0.0 && cosineValue > 0.0){
    sineValue
}else if (sineValue > 0.0 && cosineValue < 0.0){
    cosineValue
}else if (sineValue < 0.0 && cosineValue > 0.0){
    sineValue+360.0
}else{
    180-sineValue
}
/**
 * A convenience class that acts as a drop-in replacement for Rotation2d.
 * Useable everywhere that Rotation2d is, except that it can accept an Angle in it's constructor
 * and can return an angle as well.
 */
public class UnitRotation2d(public val angle: Angle = 0.0.radians){
    public constructor(x: Double, y: Double) : this(convertToAngle(x,y).radians) {}

    public val base: Rotation2d = Rotation2d.fromRadians(angle.inUnit(radians))

    public val cos: Double = cos(angle)
    public val sin: Double = sin(angle)
    public val tan: Double = tan(angle)


    public operator fun unaryMinus(): UnitRotation2d = UnitRotation2d(-angle)
    public operator fun plus(other: UnitRotation2d): UnitRotation2d = base.plus(other.base).inUnitForm()
    public operator fun minus(other: UnitRotation2d): UnitRotation2d = base.minus(other.base).inUnitForm()
    public operator fun times(scalar: Double): UnitRotation2d = base.times(scalar).inUnitForm()
    public operator fun div(scalar: Double): UnitRotation2d = base.div(scalar).inUnitForm()

    public fun rotateBy(rotation: UnitRotation2d): UnitRotation2d = base.rotateBy(rotation.base).inUnitForm()
    public fun interpolate(endValue: UnitRotation2d,t: Double): UnitRotation2d = base.interpolate(endValue.base,t).inUnitForm()






}
/*
val operatorController = chargerController(port = 1){

    val leftPowerScale = 5

    val drivePower = CurvatureOutput()

    aButton{
        onClick(2) run command()
        onClick() run command()
    }

    rightTriggerButton(threshold = 0.5){
        onHold() run command()
        whileTrue() run command()
    }
}

operatorController.configureButtonBindings()
 */