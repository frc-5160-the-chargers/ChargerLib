package frc.chargers.hardware.sensors.cameras

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.cameraserver.CameraServer

public class DriverCameraView(
    public val totalDriverCams: Int = 1,
    public val defaultResWidth: Int,
    public val defaultResHeight: Int
){
    private var driverCameras = List(totalDriverCams){ i -> CameraServer.startAutomaticCapture(i)}
    private val cameraSelector: NetworkTableEntry = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection")

    init{
        for (driverCamera in driverCameras){driverCamera.setResolution(defaultResWidth,defaultResHeight)}
    }

    public fun switchToCamera(id: Int){
        cameraSelector.setString(driverCameras[id].name)
    }

    public fun setResolutionOf(id: Int, width:Int, height:Int){
        driverCameras[id].setResolution(width,height)
    }

    public fun setResolution(width:Int, height:Int){
        driverCameras.forEach{it.setResolution(width,height)}
    }
}