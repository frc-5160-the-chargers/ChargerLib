package frc.chargers.hardware.sensors.cameras


import com.batterystaple.kmeasure.quantities.*
import kotlin.math.sqrt


public interface VisionCamera3d: VisionCamera{
    /**
     * gets how far in the x direction the camera has to travel to reach the target.
     * Here, it is pointing left-right.
     */
    public val xDistance: Distance
    /**
     * gets how far in the y direction the camera has to travel to reach the target.
     * Here, it is pointing up-down.
     */
    public val yDistance: Distance

    /**
     * gets how far in the z direction the camera has to travel to reach the target.
     * Here, it is pointing forward-backward.
     */
    public val zDistance: Distance



}

public interface VisionCamera {
    /**
     * Gets whether something has a target.
     */
    public val hasTarget: Boolean
    /**
     * gets the x(left-right) angular offset from the target.
     * in the case of the limelight, it is tx. In the case of photoncamera, it's getYaw()/yaw.
     */
    public val thetaX: Angle
    /**
     * gets the y(up-down) angular offset from the target.
     * in the case of the limelight, it is tx. In the case of photoncamera, it's getYaw()/yaw.
     */
    public val thetaY: Angle

    /**
     * Gets the area of the target.
     */
    public val area: Double


    public fun getDistance(height: Distance): Distance

    public fun getDiagonalDistance(height: Distance): Distance{
        val distanceSquared = (getDistance(height) * getDistance(height)).siValue
        val heightSquared = (height * height).siValue
        return Distance(sqrt(distanceSquared + heightSquared))
    }



}