package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

// a wrapper for the limelight; makes it easier to use
public class Limelight(public val noiseFilterThreshold: Double = 0.04,
                       public val lensHeight: Distance? = null,
                       public val mountAngle: Angle? = null){
    
    private var previousValues: MutableMap<String, Double> = mutableMapOf("tx" to 0.0,"ty" to 0.0, "ta" to 0.0)

    // gets a limelight entry.
    // simply makes the limelight entries for tx, ty and ta more consistent by filtering out "noise"
    // most reliable if used in periodic loop or used in constant loop.
    public fun getEntry(entry:String): Double{
        var value: Double = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0)
        if (entry == "tx" || entry == "ty" || entry == "ta"){
            if (value- previousValues[entry]!! < noiseFilterThreshold){
                value = previousValues[entry]!!
            }else{
                previousValues[entry] = value
            }
        }
        return value
    }
    
    // sets a limelight property.
    public fun setProperty(property:String,value:Double){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).setNumber(value)
    }

    // sets a limelight pipeline. accepts int instead of double.
    public fun setPipeline(pipeline: Int){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline)
    }

    // values must range between -1 and 1.
    // usage: limelight.setCrop(xRange = arrayOf(-0.5,0.5), yRange = arrayOf(-0.5,0.5))
    public fun setCrop(xRange: List<Double> = listOf(-1.0,1.0), yRange: List<Double> = listOf(-1.0,1.0)){
        val arrayToSend: Array<Double> = arrayOf(xRange[0],xRange[1],yRange[0],yRange[1])
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(arrayToSend)
    }

    // gets an array property instead of a regular one. useful for botpose stuff.
    // note: converts to lists cuz those are better.
    public fun getListProperty(property: String): List<Double> = NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).getDoubleArray(Array<Double>(6){0.0}).toList()

    /*
    gets horizontal distance.
    How to use:
    limeight.getHorizontalDistance(5.meters)
    obviously returns in meters
     */
    public fun getHorizontalDistance(targetHeight: Distance): Double{
        if (lensHeight == null || mountAngle == null){return 0.0}
        var angleToGoal: Angle = (mountAngle + getEntry("ty").degrees)
        return (targetHeight.inUnit(meters)/ tan(angleToGoal.inUnit(radians)))
    }
    
    /*
    gets diagonal distance from lens to target.
    How to use:
    limelight.getDistance(5.meters)
    obviously returns in meters
     */
    public fun getDistance(targetHeight: Distance): Double{
        return sqrt(getHorizontalDistance(targetHeight).pow(2.0) + targetHeight.inUnit(meters).pow(2.0))
    }    

    
}
