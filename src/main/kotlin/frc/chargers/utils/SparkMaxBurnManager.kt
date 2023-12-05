package frc.chargers.utils
// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage


import edu.wpi.first.wpilibj.RobotBase
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Paths


/** Determines whether to burn SparkMax configs to flash.  */
public class SparkMaxBurnManager(
    private val buildDate: String
) {
    private val buildDateFile = "/home/lvuser/build-date.txt"
    private var shouldBurn = false
    public fun shouldBurn(): Boolean {
        return shouldBurn
    }

    public fun update() {
        if (RobotBase.isSimulation()) {
            shouldBurn = false
            return
        }
        val file = File(buildDateFile)
        if (!file.exists()) {

            // No build date file, burn flash
            shouldBurn = true
        } else {

            // Read previous build date
            var previousBuildDate = ""
            try {
                previousBuildDate = String(Files.readAllBytes(Paths.get(buildDateFile)), StandardCharsets.UTF_8)
            } catch (e: IOException) {
                e.printStackTrace()
            }
            shouldBurn = previousBuildDate != buildDate
        }
        try {
            val fileWriter = FileWriter(buildDateFile)
            fileWriter.write(buildDate)
            fileWriter.close()
        } catch (e: IOException) {
            e.printStackTrace()
        }
        if (shouldBurn) {
            println("[SparkMaxBurnManager] Build date changed, burning SparkMax flash")
        } else {
            println(
                "[SparkMaxBurnManager] Build date unchanged, will not burn SparkMax flash"
            )
        }
    }
}