package frc.chargers.hardware.inputdevices

import frc.chargers.wpilibextensions.kinematics.ChassisPowers

/**
 * A subclass of [ChargerController] which has the capacity to control a differential drivetrain.
 *
 */
public class CurvatureDriveController(
    port: Int,
    private val getForwardsPower: ChargerController.() -> Double,
    private val getRotationPower: ChargerController.() -> Double,
    private val getTurboPower: ChargerController.() -> Double = {1.0},
    private val getPrecisionPower: ChargerController.() -> Double = {1.0},
    deadband: Double,
    defaultAxisThreshold: Double
): ChargerController(port, deadband, defaultAxisThreshold){

    public companion object{
        /**
         * Creates a [CurvatureDriveController] with default bindings:
         *
         * left Y goes straight, left X goes sideways, right X rotates,
         *
         * leftTriggerAxis triggers turbo mode, rightTriggerAxis triggers precision mode.
         */
        public fun fromDefaultBindings(
            port: Int,
            driveMultiplier: Double = 1.0,
            rotationMultiplier: Double = 1.0,
            turboModeMultiplierRange: ClosedRange<Double> = 0.0..1.0,
            precisionModeDividerRange: ClosedRange<Double> = 0.0..1.0,
            deadband: Double = 0.0,
            defaultAxisThreshold: Double = 0.5
        ): CurvatureDriveController =
            CurvatureDriveController(
                port,
                {leftY * driveMultiplier},
                {rightX * rotationMultiplier},
                {rightTriggerAxis.mapTriggerValue(turboModeMultiplierRange)},
                {1/leftTriggerAxis.mapTriggerValue(precisionModeDividerRange)},
                deadband,
                defaultAxisThreshold
            )
    }
    public val curvatureOutput: ChassisPowers
        get(){
            val multiplier = getTurboPower() * getPrecisionPower()
            return ChassisPowers(
                xPower = getForwardsPower().withDeadband() * multiplier,
                rotationPower = getRotationPower().withDeadband() * multiplier
            )
        }
}

