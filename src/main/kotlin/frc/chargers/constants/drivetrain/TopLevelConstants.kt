package frc.chargers.constants.drivetrain

import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.grams
import com.batterystaple.kmeasure.units.kilo
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.utils.math.units.Inertia
import frc.chargers.utils.math.units.times

@PublishedApi
internal val DEFAULT_MAX_MODULE_SPEED: Velocity = 4.5.ofUnit(meters / seconds)

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

@PublishedApi
internal val DEFAULT_SWERVE_TURN_INERTIA: Inertia = 0.025.ofUnit(kilo.grams * meters * meters)

@PublishedApi
internal val DEFAULT_SWERVE_DRIVE_INERTIA: Inertia = 0.004096955.ofUnit(kilo.grams * meters * meters)