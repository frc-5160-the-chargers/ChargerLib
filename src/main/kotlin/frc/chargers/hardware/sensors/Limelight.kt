package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Area
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
    private var nt: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight")

    // NOTE: This part is different than the networktables api!!!!!!
    public var pipeline: Int
        get() = nt.getEntry("getpipe").getDouble(0.0) as Int
        set(value){
            nt.getEntry("pipeline").setNumber(value)
        }

    // alternative to use base tv.
    public val hasTarget: Boolean
        get() = nt.getEntry("tv").getDouble(0.0) == 1.0

    private var previousTX: Double = 0.0
    private var previousTY: Double = 0.0
    private var previousTA: Double = 0.0

    public val tx: Angle
        get(){
            var value: Double = nt.getEntry("tx").getDouble(0.0)

            if (value - previousTX < noiseFilterThreshold){
                return ((value + previousTX)/2).degrees
            }else{
                previousTX = value
                return value.degrees
            }
        }
    public val ty: Angle
        get(){
            var value: Double = nt.getEntry("ty").getDouble(0.0)

            if (value - previousTY < noiseFilterThreshold){
                return ((value + previousTY)/2).degrees
            }else{
                previousTY = value
                return value.degrees
            }
        }
    public val ta: Double
        get(){
            var value: Double = nt.getEntry("ta").getDouble(0.0)

            if (value - previousTA < noiseFilterThreshold){
                return (value + previousTA)/2
            }else{
                previousTA = value
                return value
            }
        }
    public val tv: Double
        get() = nt.getEntry("tv").getDouble(0.0)
    public val tl: Double
        get() = nt.getEntry("tl").getDouble(0.0)
    public val cl: Double
        get() = nt.getEntry("cl").getDouble(0.0)
    public val tshort: Double
        get() = nt.getEntry("tshort").getDouble(0.0)
    public val tlong: Double
        get() = nt.getEntry("tlong").getDouble(0.0)
    public val thor: Double
        get() = nt.getEntry("thor").getDouble(0.0)
    public val tvert: Double
        get() = nt.getEntry("tvert").getDouble(0.0)
    public val json: Double
        get() = nt.getEntry("json").getDouble(0.0)
    public val tclass: Double
        get() = nt.getEntry("tclass").getDouble(0.0)
    public val tc: Double
        get() = nt.getEntry("tc").getDouble(0.0)


    // NOTE: the array entry is auto converted into a list here, cuz lists are better
    public val botpose: List<Double>
        get() = nt.getEntry("botpose").getDoubleArray(Array(6){0.0}).toList()

    public val botpose_wpiblue: List<Double>
        get() = nt.getEntry("botpose_wpiblue").getDoubleArray(Array(6){0.0}).toList()

    public val botpose_wpired: List<Double>
        get() = nt.getEntry("botpose_wpired").getDoubleArray(Array(6){0.0}).toList()

    public val camerapose_targetspace: List<Double>
        get() = nt.getEntry("botpose").getDoubleArray(Array(6){0.0}).toList()

    public val targetpose_cameraspace: List<Double>
        get() = nt.getEntry("botpose").getDoubleArray(Array(6){0.0}).toList()

    public val targetpose_robotspace: List<Double>
        get() = nt.getEntry("botpose").getDoubleArray(Array(6){0.0}).toList()

    public val camerapose_robotspace: List<Double>
        get() = nt.getEntry("botpose").getDoubleArray(Array(6){0.0}).toList()

    public val tid: List<Double>
        get() = nt.getEntry("tid").getDoubleArray(Array(6){0.0}).toList()


    public var ledMode: Int = 0
        set(value){
            field = value
            nt.getEntry("ledMode").setNumber(value)
        }

    public var camMode: Int = 0
        set(value){
            field = value
            nt.getEntry("camMode").setNumber(value)
        }

    public var stream: Int = 0
        set(value){
            field = value
            nt.getEntry("stream").setNumber(value)

        }

    public var crop: List<Double> = listOf(-1.0,1.0,-1.0,1.0)
        set(value){
            field = value
            nt.getEntry("crop").setDoubleArray(value.toTypedArray())
        }

    public fun snapshot(){
        nt.getEntry("snapshot").setNumber(1)
    }

    public fun resetSnapshotMode(){
        nt.getEntry("snapshot").setNumber(1)
    }


    public fun getRawEntry(entry:String): Double = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0)



    
    // sets a limelight property.
    public fun setRawProperty(property:String,value:Double){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).setNumber(value)
    }



    // gets an array property instead of a regular one. useful for botpose stuff.
    // note: converts to lists cuz those are better.
    public fun getRawListProperty(property: String): List<Double> = NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).getDoubleArray(Array<Double>(6){0.0}).toList()

    /*
    gets horizontal distance.
    How to use:
    limeight.getHorizontalDistance(5.meters).inUnit(meters)
    obviously returns in meters
     */
    public fun getHorizontalDistance(targetHeight: Distance): Distance{
        if (lensHeight == null || mountAngle == null){return 0.0.meters}
        var angleToGoal: Angle = (mountAngle + ty)
        return (targetHeight.inUnit(meters)/ tan(angleToGoal.inUnit(radians))).meters
    }
    
    /*
    gets diagonal distance from lens to target.
    How to use:
    limelight.getDistance(5.meters).inUnit(meters)
    obviously returns in meters
     */
    public fun getDistance(targetHeight: Distance): Distance{
        return sqrt(getHorizontalDistance(targetHeight).inUnit(meters).pow(2.0) + targetHeight.inUnit(meters).pow(2.0)).meters
    }    

    
}
