package frc.chargers.hardware.sensors.imu

import frc.chargers.advantagekitextensions.LoggableInputsProvider

/*
The classes below manage logging + log replay for all classes that implement the IMU interface.
 */

internal val ImuLog: LoggableInputsProvider = LoggableInputsProvider(
    namespace = "IMU"
)

internal val GyroLog: LoggableInputsProvider = LoggableInputsProvider(
    namespace = "IMU/gyroscope"
)

internal val SpeedometerLog: LoggableInputsProvider = LoggableInputsProvider(
    namespace = "IMU/speedometer"
)

internal val AccelerometerLog: LoggableInputsProvider = LoggableInputsProvider(
    namespace = "IMU/accelerometer"
)
