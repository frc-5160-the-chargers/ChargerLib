package frc.chargers.framework

/**
 * A basic data logger that avoids spamming the console with messages.
 */
public object ConsoleLogger {

    private val logData: MutableMap<String, Int> = mutableMapOf()

    private var totalLoops: Int = 0

    init{
        ChargerRobot.addToPeriodicLoop { totalLoops++ }
    }


    public fun write(data: String){
        if (data in logData.keys){
            logData[data] = logData[data]!! + 1
        }else{
            logData[data] = 1
        }
    }


    public fun print(){
        println("Console Logger Output: ")
        for (log in logData.keys){
            println(log + "(Total logs: " + logData[log] + "; Average Logs/Loop: " + (logData[log]!!/totalLoops) + ")")
        }
    }

}