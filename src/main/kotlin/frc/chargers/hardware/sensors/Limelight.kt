package frc.chargers.sensors;

import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance

// a wrapper for the limelight; makes it easier to use
class Limelight(noiseFilterThreshold: Double = 0.04, lensHeight: Distance? = null, mountAngle: Angle? = null){
    
    public var previousValues = mapOf("tx" to 0.0,"ty" to 0.0, "ta" to 0.0)

    // gets a limelight entry.
    // simply makes the limelight entries for tx, ty and ta more consistent by filtering out "noise"
    // most reliable if used in periodic loop or used in constant loop.
    fun getEntry(entry:String): Double{
        var value: Double = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0)
        if (entry == "tx" || entry == "ty" || entry == "ta"){
            if (value-previousValues[entry] < noiseFilterThreshold){
                value = entry
            }else{
                previousValues[entry] = value
            }
        }
        return value
    }
    
    // sets a limelight property.
    fun setProperty(property:String,value:Double){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).setNumber(value)
    }

    // sets a limelight pipeline. accepts int instead of double.
    fun setPipeline(pipeline: Int){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline)
    }

    // values must range between -1 and 1.
    // usage: limelight.setCrop(xRange = arrayOf(-0.5,0.5), yRange = arrayOf(-0.5,0.5))
    fun setCrop(xRange: Array<Double> = arrayOf(-1.0,1.0), yRange: Array<double> = arrayOf(-1.0,1.0)){
        private val arrayToSend: Array<Double> = arrayOf(xRange[0],xRange[1],yRange[0],yRange[1])
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(arrayToSend)
    }

    // gets an array property instead of a regular one. useful for botpose stuff.
    fun getArrayProperty(property: String): Array<Double> = NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).getDoubleArray(Array(6))

    /*
    gets horizontal distance.
    How to use:
    limeight.getHorizontalDistance(5.meters)
    obviously returns in meters
     */
    fun getHorizontalDistance(targetHeight: Distance): Double{
        if (lensHeight == null || mountAngle == null){return}
        var angleToGoal: Angle = (mountAngle + getEntry("ty").inUnit(degrees))
        return (targetHeight.meters/Math.tan(angleToGoal.radians))
    }
    
    /*
    gets diagonal distance from lens to target.
    How to use:
    limelight.getDistance(5.meters)
    obviously returns in meters
     */
    fun getDistance(targetHeight: Distance): Double{
        return Math.sqrt(getHorizontalDistance(targetHeight).pow(2.0) + targetHeight.meters.pow(2.0))
    }    

    
}
