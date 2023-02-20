package frc.chargers.hardware.inputdevices

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.Button

/**
 * This class allows you to trigger commands from an analog input on
 * a joystick (sich as the triggers - Axis 3).
 *
 *
 * The following example code placed in OI class turns axis 3 into two buttons:
 * ----------------------------------------------------------------------------
 * //Create an AnalogButton for each trigger
 * int joystickChannel = 1;
 * public JoystickAnalogButton TriggerR = new JoystickAnalogButton(joystickChannel, 3, -0.5),
 * TriggerL = new JoystickAnalogButton(joystickChannel, 3, 0.5)
 *
 * //Link the buttons to commands
 * TriggerR.whenPressed(new ExampleCommand1());
 * TriggerL.whenPressed(new ExampleCommand2())
 *
 * Note that since both buttons are on the same Axis channel, they cannot be
 * pressed simultaneously. One trigger will negate the other and neither will
 * look pressed. So plan your controls accordingly.
 *
 * @author James@team2168.org
 * @author Rohen Giralt
 */
public class JoystickAnalogButton : Button {
    public val joystick: GenericHID
    public val axisNumber: Int

    /**
     * The value above which triggers should occur (for positive thresholds)
     * or below which triggers should occur (for negative thresholds)
     * The default threshold value is 0.5
     */
    public var threshold: Double = 0.5

    /**
     * Create a button for triggering commands off a joystick's analog axis
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number
     */
    public constructor(joystick: GenericHID, axisNumber: Int) {
        this.joystick = joystick
        this.axisNumber = axisNumber
    }

    /**
     * Create a button for triggering commands off a joystick's analog axis
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number
     * @param threshold The threshold to trigger above (positive) or below (negative)
     */
    public constructor(joystick: GenericHID, axisNumber: Int, threshold: Double) {
        this.joystick = joystick
        this.axisNumber = axisNumber
        this.threshold = threshold
    }

    override fun get(): Boolean {
        return if (threshold < 0) {
            joystick.getRawAxis(axisNumber) < threshold //Return true if axis value is less than negative threshold
        } else {
            joystick.getRawAxis(axisNumber) > threshold //Return true if axis value is greater than positive threshold
        }
    }
}