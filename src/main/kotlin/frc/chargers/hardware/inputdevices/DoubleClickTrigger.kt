package frc.chargers.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger

public fun Trigger.withDoubleClick(clickTimeout: Time = 0.02.seconds): Trigger{
    val tracker = DoubleClickTracker(this, clickTimeout)
    return Trigger(tracker::invoke)
}

public fun Trigger.onDoubleClick(clickTimeout: Time = 0.02.seconds, command: Command){
    withDoubleClick(clickTimeout).onTrue(command)
}


/*
Inspired off of 6328's class
 */
private class DoubleClickTracker(private val baseTrigger: Trigger, private val clickTimeout: Time): () -> Boolean{

    private enum class DoubleClickState{
        UNPRESSED, FIRST_PRESS, FIRST_RELEASE, SECOND_PRESS
    }

    private var state = DoubleClickState.UNPRESSED

    private val clickTimer = Timer()


    override fun invoke(): Boolean {
        when(state){
            DoubleClickState.UNPRESSED -> {
                if (baseTrigger.asBoolean){
                    state = DoubleClickState.FIRST_PRESS
                    clickTimer.reset()
                    // ChargerLib extension property
                    clickTimer.start()
                }
            }

            DoubleClickState.FIRST_PRESS -> {
                if (timeLimitReached()){
                    reset()
                }else{
                    state = DoubleClickState.FIRST_RELEASE
                }
            }

            DoubleClickState.FIRST_RELEASE -> {
                if (baseTrigger.asBoolean){
                    state = DoubleClickState.SECOND_PRESS
                }else if (timeLimitReached()){
                    reset()
                }
            }

            DoubleClickState.SECOND_PRESS -> {
                if (!baseTrigger.asBoolean){
                    reset()
                }
            }
        }
        return state == DoubleClickState.SECOND_PRESS
    }

    private fun timeLimitReached() = clickTimer.hasElapsed(clickTimeout.inUnit(seconds))

    private fun reset(){
        clickTimer.stop()
        state = DoubleClickState.UNPRESSED
    }
}