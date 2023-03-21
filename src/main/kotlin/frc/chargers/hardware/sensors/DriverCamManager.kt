package frc.chargers.hardware.sensors
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.cameraserver.CameraServer

public class DriverCamManager(
    public val totalDriverCams: Int = 0,
    public val defaultResWidth: Int = 720,
    public val defaultResHeight: Int = 1280){
    internal var driverCameras = Array(totalDriverCams){i -> CameraServer.startAutomaticCapture(i)}.toList()
    internal val cameraSelector: NetworkTableEntry = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection")

    init{
        driverCameras = Array(totalDriverCams){i -> CameraServer.startAutomaticCapture(i)}.toList()
        for (i in driverCameras){i.setResolution(defaultResWidth,defaultResHeight)}
    }

    public fun switchToCamera(id: Int){
        cameraSelector.setString(driverCameras[id].getName())
    }

    public fun setDriverCamResolution(id: Int, width:Int, height:Int){
        driverCameras[id].setResolution(width,height)
    }
}
