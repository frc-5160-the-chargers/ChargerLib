package frc.robot.hardware.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import frc.robot.hardware.interfaces.Encoder
import frc.robot.hardware.interfaces.EncoderMotorController
import frc.robot.hardware.interfaces.MotorConfigurable
import frc.robot.hardware.interfaces.MotorConfiguration

/**
 * A [MotorControllerGroup] that aggregates the encoders
 * of its [MotorController]s in addition to their speeds.
 */
public open class NonConfigurableEncoderMotorControllerGroup(protected vararg val motorControllers: MotorController, override val encoder: Encoder) : MotorControllerGroup(motorControllers), EncoderMotorController {
    public constructor(vararg encoderMotorControllers: EncoderMotorController) : this(*encoderMotorControllers, encoder = AverageEncoder(*encoderMotorControllers))
}

/**
 * Similar to [NonConfigurableEncoderMotorControllerGroup],
 * a [MotorControllerGroup] that aggregates the encoders
 * of its [MotorController]s in addition to their speeds.
 * However, also allows for group configuration of all
 * its MotorControllers at once.
 *
 * @see NonConfigurableEncoderMotorControllerGroup
 */
public open class EncoderMotorControllerGroup<C : MotorConfiguration> private constructor( // Actual constructor private; see fake constructors in companion object
    vararg motorControllers: MotorController, encoder: Encoder
) : NonConfigurableEncoderMotorControllerGroup(*motorControllers, encoder = encoder), MotorConfigurable<C> {

    override fun configure(configuration: C) {
        motorControllers.forEach { motorController ->
            @Suppress("UNCHECKED_CAST") // Allowed because motorController is always set to an instance of
                                                // MotorConfigurable<C> in the fake constructors
            motorController as MotorConfigurable<C>

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
        public operator fun <M, C : MotorConfiguration> invoke(vararg motorControllers: M, encoder: Encoder): EncoderMotorControllerGroup<C> where M : MotorController, M : MotorConfigurable<C> =
            EncoderMotorControllerGroup(*motorControllers, encoder = encoder)

        public operator fun <M, C : MotorConfiguration> invoke(vararg encoderMotorControllers: M) : EncoderMotorControllerGroup<C> where M : EncoderMotorController, M : MotorConfigurable<C> =
            invoke(*encoderMotorControllers, encoder = AverageEncoder(*encoderMotorControllers))
    }
}