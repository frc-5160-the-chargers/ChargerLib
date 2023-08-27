package frc.chargers.hardware.sensors.cameras


import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance




public interface VisionCamera3d: VisionCamera{
    /**
     * gets how far in the x direction the camera has to travel to reach the target.
     * Here, it is pointing left-right.
     */
    public val xDistanceOffset: Distance
    /**
     * gets how far in the y direction the camera has to travel to reach the target.
     * Here, it is pointing up-down.
     */
    public val yDistanceOffset: Distance

    /**
     * gets how far in the z direction the camera has to travel to reach the target.
     * Here, it is pointing forward-backward.
     */
    public val zDistanceOffset: Distance


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
    public val xAngularOffset: Angle
    /**
     * gets the y(up-down) angular offset from the target.
     * in the case of the limelight, it is tx. In the case of photoncamera, it's getYaw()/yaw.
     */
    public val yAngularOffset: Angle

    /**
     * Gets the area of the target.
     */
    public val area: Double


    public fun getDistance(height: Distance): Distance

    public fun getDiagonalDistance(height: Distance): Distance


}