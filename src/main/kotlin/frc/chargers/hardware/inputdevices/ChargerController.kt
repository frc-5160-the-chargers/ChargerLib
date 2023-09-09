package frc.chargers.hardware.inputdevices

import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.utils.math.mapBetweenRanges
import kotlin.math.abs

public typealias TriggerValue = Double

/**
 * The base of all input devices within ChargerLib. Based off of [CommandXboxController].
 *
 * Example of usage of this class:
 *
 * ```
 * val controller = ChargerController(port = 0)
 *
 * controller.apply{
 *      a{
 *          onTrue(command)
 *          whileTrue(command)
 *      }
 * }
 * ```
 */
public open class ChargerController(
    port: Int,
    public val deadband: Double = 0.0,
    public val defaultAxisThreshold: Double = 0.5
): CommandXboxController(port) {

    protected fun TriggerValue.withDeadband(): Double =
        if (abs(this) <= deadband) { 0.0 } else { this }

    public fun TriggerValue.mapTo(to: ClosedRange<Double>): Double =
        mapBetweenRanges(0.0..1.0, to)

    public inline fun a(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            a().apply(bindings)
        }else{
            a(loop).apply(bindings)
        }
    }
    public inline fun b(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            b().apply(bindings)
        }else{
            b(loop).apply(bindings)
        }
    }
    public inline fun x(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            x().apply(bindings)
        }else{
            x(loop).apply(bindings)
        }
    }
    public inline fun y(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            y().apply(bindings)
        }else{
            y(loop).apply(bindings)
        }
    }

    public inline fun leftTrigger(threshold: Double = defaultAxisThreshold, loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            leftTrigger(threshold).apply(bindings)
        }else{
            leftTrigger(loop,threshold).apply(bindings)
        }
    }

    public inline fun rightTrigger(threshold: Double = defaultAxisThreshold, loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            rightTrigger(threshold).apply(bindings)
        }else{
            rightTrigger(threshold,loop).apply(bindings)
        }
    }


    public inline fun leftBumper(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            leftBumper().apply(bindings)
        }else{
            leftBumper(loop).apply(bindings)
        }
    }

    public inline fun rightBumper(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            rightBumper().apply(bindings)
        }else{
            rightBumper(loop).apply(bindings)
        }
    }

    public inline fun leftStick(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            leftStick().apply(bindings)
        }else{
            leftStick(loop).apply(bindings)
        }
    }

    public inline fun rightStick(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            rightStick().apply(bindings)
        }else{
            rightStick(loop).apply(bindings)
        }
    }

    public inline fun startButton(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            start().apply(bindings)
        }else{
            start(loop).apply(bindings)
        }
    }

    public inline fun backButton(loop: EventLoop? = null, bindings: Trigger.() -> Unit){
        if (loop == null){
            back().apply(bindings)
        }else{
            back(loop).apply(bindings)
        }
    }
}




