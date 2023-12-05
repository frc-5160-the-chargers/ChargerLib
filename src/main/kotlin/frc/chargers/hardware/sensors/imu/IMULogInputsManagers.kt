package frc.chargers.hardware.sensors.imu

import frc.chargers.advantagekitextensions.LoggableInputsProvider

/*
The classes below manage logging + log replay for all classes that implement the IMU interface.
 */

internal val IMU_INPUTS: LoggableInputsProvider = LoggableInputsProvider(logNamespace = "IMU")

internal val GYRO_INPUTS: LoggableInputsProvider = LoggableInputsProvider(logNamespace = "IMU/gyroscope")

internal val SPEEDOMETER_INPUTS: LoggableInputsProvider = LoggableInputsProvider(logNamespace = "IMU/speedometer")

internal val ACCELEROMETER_INPUTS: LoggableInputsProvider = LoggableInputsProvider(logNamespace = "IMU/accelerometer")

internal val COMPASS_INPUTS: LoggableInputsProvider = LoggableInputsProvider(logNamespace = "IMU/compass")