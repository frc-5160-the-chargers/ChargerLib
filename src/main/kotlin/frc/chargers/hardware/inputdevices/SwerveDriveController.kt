package frc.chargers.hardware.inputdevices

import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * An extension of [ChargerController] that provides tools to assist with controlling a swerve drivetrain.
 */
public class SwerveDriveController(
    port: Int,
    private val getForwardsPower: SwerveDriveController.() -> Double,
    private val getStrafePower: SwerveDriveController.() -> Double,
    private val getRotationPower: SwerveDriveController.() -> Double,
    private val getTurboPower: SwerveDriveController.() -> Double = {1.0},
    private val getPrecisionPower: SwerveDriveController.() -> Double = {1.0},
    deadband: Double = 0.0,
    defaultAxisThreshold: Double = 0.5
): ChargerController(port, deadband, defaultAxisThreshold) {

    public companion object{
        /**
         * Creates a [SwerveDriveController] with default bindings:
         *
         * left Y goes straight, left X goes sideways, right X rotates,
         *
         * leftTriggerAxis triggers turbo mode, rightTriggerAxis triggers precision mode.
         */
        public fun fromDefaultBindings(
            port: Int,
            driveMultiplier: Double = 1.0,
            rotationMultiplier: Double = 1.0,
            turboModeMultiplierRange: ClosedRange<Double> = 1.0..1.0,
            precisionModeDividerRange: ClosedRange<Double> = 1.0..1.0,
            deadband: Double = 0.0,
            defaultAxisThreshold: Double = 0.5
        ): SwerveDriveController =
            SwerveDriveController(
                port,
                {leftY * driveMultiplier},
                {leftX * driveMultiplier},
                {rightX * rotationMultiplier},
                {rightTriggerAxis.mapTriggerValue(turboModeMultiplierRange)},
                {1/leftTriggerAxis.mapTriggerValue(precisionModeDividerRange)},
                deadband,
                defaultAxisThreshold
            )
    }
    public val swerveOutput: ChassisSpeeds
        get(){
            val multiplier = getTurboPower() * getPrecisionPower()
            return ChassisSpeeds(
                getForwardsPower().withDeadband() * multiplier,
                getStrafePower().withDeadband() * multiplier,
                getRotationPower().withDeadband() * multiplier
            )
        }
}