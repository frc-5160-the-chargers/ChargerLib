package frc.chargers.hardware.sensors
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.cameraserver.CameraServer

class DriverCamManager(
    totalDriverCams: Int = 0, 
    defaultResWidth: Int = 720, 
    defaultResHeight: Int = 1280){
    internal val driverCameras = Array(totalDriverCams){i -> CameraServer.startAutomaticCapture(i)}.toList()
    internal val cameraSelector: NetworkTableEntry = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection")

    init{
        driverCameras = Array(totalDriverCams){i -> CameraServer.startAutomaticCapture(i)}.toList()
        driverCameras.forEach(::setResolution(defaultResWidth,defaultResHeight))
    }

    fun switchToCamera(id: Int){
        cameraSelector.setString(driverCameras[id].getName())
    }

    fun setDriverCamResolution(id: Int,width:Int,height:Int){
        driverCameras[id].setResolution(width,height)
    }
}
