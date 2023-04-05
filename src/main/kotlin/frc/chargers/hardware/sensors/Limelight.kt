package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import frc.chargers.javaUtils.LimelightHelpers
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan
import kotlin.math.abs



// a wrapper for the limelight; makes it easier to use
public class Limelight(public val name: String = "limelight",
                       public val noiseFilterThreshold: Double = 0.04,
                       public val lensHeight: Distance? = null,
                       public val mountAngle: Angle? = null){

    private var nt: NetworkTable = NetworkTableInstance.getDefault().getTable(name)

    // designed to inter-operate with the limelightLib "library"(literally just copy-pasted code)
    // how to use: val results = limelight.fiducialResults
    // val targetNumber: Int = results.length
    public val fiducialResults: List<LimelightHelpers.LimelightTarget_Fiducial>
        get() = LimelightHelpers.getLatestResults(name).targetingResults.targets_Fiducials.toList()

    public val retroreflectiveResults: List<LimelightHelpers.LimelightTarget_Retro>
        get() = LimelightHelpers.getLatestResults(name).targetingResults.targets_Retro.toList()

    public val neuralClassifierResults: List<LimelightHelpers.LimelightTarget_Classifier>
        get() = LimelightHelpers.getLatestResults(name).targetingResults.targets_Classifier.toList()

    public val neuralDetectorResults: List<LimelightHelpers.LimelightTarget_Detector>
        get() = LimelightHelpers.getLatestResults(name).targetingResults.targets_Detector.toList()





    // NOTE: This part is different than the networktables api!!!!!!
    public var pipeline: Int
        get() = nt.getEntry("getpipe").getDouble(0.0).toInt()
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
            val value: Double = nt.getEntry("tx").getDouble(0.0)

            return if (abs(value - previousTX) < noiseFilterThreshold){
                previousTX.degrees
            }else{
                previousTX = value
                value.degrees
            }
        }
    public val ty: Angle
        get(){
            val value: Double = nt.getEntry("ty").getDouble(0.0)

            return if (abs(value - previousTY) < noiseFilterThreshold){
                previousTY.degrees
            }else{
                previousTY = value
                value.degrees
            }
        }
    public val ta: Double
        get(){
            val value: Double = nt.getEntry("ta").getDouble(0.0)

            return if (abs(value - previousTA) < noiseFilterThreshold){
                previousTA
            }else{
                previousTA = value
                value
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
    public val json: String
        get() = nt.getEntry("json").getString("")
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
        get() = nt.getEntry("camerapose_targetspace").getDoubleArray(Array(6){0.0}).toList()

    public val targetpose_cameraspace: List<Double>
        get() = nt.getEntry("targetpose_cameraspace").getDoubleArray(Array(6){0.0}).toList()

    public val targetpose_robotspace: List<Double>
        get() = nt.getEntry("targetpose_robotspace").getDoubleArray(Array(6){0.0}).toList()

    public val camerapose_robotspace: List<Double>
        get() = nt.getEntry("camerapose_robotspace").getDoubleArray(Array(6){0.0}).toList()

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



    // gets a limelight entry. Only for use if limelight adds new unsupported features.
    public fun getGenericEntry(entry:String): Double = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0)


    
    // sets a limelight property. Only for use if limelight adds new unsupported features.
    public fun setGenericProperty(property:String,value:Double){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).setNumber(value)
    }



    // gets a limelight list property. only for use if limelight adds new unsupported features.
    public fun getGenericListProperty(property: String): List<Double> = NetworkTableInstance.getDefault().getTable("limelight").getEntry(property).getDoubleArray(Array<Double>(6){0.0}).toList()

    /*
    gets horizontal distance.
    How to use:
    limeight.getHorizontalDistance(5.meters).inUnit(meters)
    obviously returns in meters
     */
    public fun getHorizontalDistance(targetHeight: Distance): Distance{
        if (lensHeight == null || mountAngle == null){return 0.0.meters}
        val angleToGoal: Angle = (mountAngle + ty)
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

    public fun getDistanceWithAreaRatio(ratio: Distance): Distance{
        return (ta * ratio.inUnit(meters)).meters
    }

    
}
