package frc.chargers.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.sensors.encoders.AverageEncoder
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * A [MotorControllerGroup] that aggregates the encoders
 * of its [MotorController]s in addition to their speeds.
 */
public open class NonConfigurableEncoderMotorControllerGroup(
    protected vararg val motorControllers: MotorController,
    override val encoder: Encoder
) : MotorControllerGroup(motorControllers), EncoderMotorController {
    public constructor(vararg encoderMotorControllers: EncoderMotorController) : this(*encoderMotorControllers, encoder = AverageEncoder(*encoderMotorControllers))

    // getInverted()/setInverted() in the WPILib MotorControllerGroup class
    // inverts motors by storing whether the MotorControllerGroup is currently inverted,
    // and, if it is, multiplying all motor powers by -1 whenever they're set.
    // This works fine when not dealing with encoders, as it makes all the motors
    // run backwards when setInverted(true) is called.
    // However, since setInverted() is never called on the individual MotorControllers
    // making up the group, they don't adjust their encoder direction properly, suddenly
    // meaning that a positive motor power makes their encoder value more negative.
    // Thus, getInverted() and setInverted() are overridden here in order to properly invert
    // the underlying MotorControllers and their encoders.
    override fun setInverted(isInverted: Boolean) {
        if (isInverted == this.isInverted) return // If motors already in the correct state, do nothing

        toggleMotorControllerInverts() // Otherwise, toggle all the motors from inverted to not inverted (or vice versa)
        this.isInverted = isInverted
    }

    private var isInverted: Boolean = false
    override fun getInverted(): Boolean = isInverted

    private fun toggleMotorControllerInverts() {
        for (motor in motorControllers) {
            motor.inverted = !motor.inverted // Toggle whether this motor is inverted
        }
    }
}

/**
 * Similar to [NonConfigurableEncoderMotorControllerGroup],
 * a [MotorControllerGroup] that aggregates the encoders
 * of its [MotorController]s in addition to their speeds.
 * However, it also allows for group configuration of all
 * its MotorControllers at once.
 *
 * @see NonConfigurableEncoderMotorControllerGroup
 */
public open class EncoderMotorControllerGroup<C : HardwareConfiguration> private constructor( // Actual constructor private; see fake constructors in companion object
    vararg motorControllers: MotorController, encoder: Encoder
) : NonConfigurableEncoderMotorControllerGroup(*motorControllers, encoder = encoder), HardwareConfigurable<C> {

    override fun configure(configuration: C) {
        motorControllers.forEach { motorController ->
            @Suppress("UNCHECKED_CAST") // Allowed because motorController is always set to an instance of
                                                // MotorConfigurable<C> in the fake constructors

            motorController as HardwareConfigurable<C> // Tell the compiler we know motorController is MotorConfigurable<C>

            motorController.configure(configuration)
        }
    }

    /**
     * Contains fake constructors in order to ensure that EncoderMotorControllerGroups
     * are always both MotorControllers and MotorConfigurable<C>, which isn't possible
     * to ensure in a regular constructor.
     *
     * These functions add the "invoke" operator to the companion in order to create
     * a fake constructor. See [here](https://medium.com/@pablisco/companion-factory-methods-in-kotlin-e2eeb1e87f1b)
     * for more information on this technique.
     */
    public companion object {
        public operator fun <M, C : HardwareConfiguration> invoke(vararg motorControllers: M, encoder: Encoder, configuration: C? = null): EncoderMotorControllerGroup<C> where M : MotorController, M : HardwareConfigurable<C> =
            EncoderMotorControllerGroup<C>(*motorControllers, encoder = encoder)
                .apply {
                    if (configuration != null) {
                        configure(configuration)
                    }
                }

        public operator fun <M, C : HardwareConfiguration> invoke(vararg encoderMotorControllers: M, configuration: C? = null) : EncoderMotorControllerGroup<C> where M : EncoderMotorController, M : HardwareConfigurable<C> =
            invoke(*encoderMotorControllers, encoder = AverageEncoder(*encoderMotorControllers))
                .apply {
                    if (configuration != null) {
                        configure(configuration)
                    }
                }
    }
}