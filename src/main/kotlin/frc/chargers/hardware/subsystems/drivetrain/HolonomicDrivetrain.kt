package frc.chargers.hardware.subsystems.drivetrain

import com.batterystaple.kmeasure.quantities.Angle


/**
 * An interface used for swerve drive.
 */
public interface HolonomicDrivetrain {

    /**
     * Drives the robot using an xPower(forward), yPower(side-to-side) and rotationPower(rotates).
     */
    public fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double)

    /**
     * Drives the robot with a specified power, at a specific angle.
     */
    public fun directionalDrive(xPower: Double, angle: Angle)

    public fun rotateInPlace(power: Double)

    /**
     * Stops the robot.
     */
    public fun stop()

    
}