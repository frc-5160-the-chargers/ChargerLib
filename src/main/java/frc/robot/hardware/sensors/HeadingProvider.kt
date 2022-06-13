package frc.robot.hardware.sensors

import com.batterystaple.kmeasure.Angle

/**
 * An interface representing a device capable of calculating the heading of a robot (the direction it is facing).
 */
public interface HeadingProvider {
    /**
     * The heading of the robot, with the clockwise direction as positive.
     *
     * Note that this value may be any arbitrary angle, including angles
     * outside the range of [-360º-360º]. This ensures that this value changes
     * continuously, not jumping sharply from 359º to 0º. This also implies
     * that all counterclockwise rotations will increase this value; otherwise,
     * a (say) 2º rotation at position 359º would give 1º, a smaller value
     * due to wrapping around.
     */
    public val heading: Angle
}
