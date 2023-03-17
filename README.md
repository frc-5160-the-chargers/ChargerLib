# ChargerLib

ChargerLib is the backbone of all FRC code for the FRC 5160 team(at least so far). It adds a lot of useful features that make robot coding easier, simpler and better overall.
Unfortunately, there's no docs for chargerlib so far; however, this readme will give you the basics about it.


# Gradle setup in your FRC projects

In order to use ChargerLib, you're going to have to add lines of code into the build.gradle and settings.gradle parts of your FRC ROBOT code. 
You do not have to edit chargerlib at all; it should always be set up to work.

### 1. In your build.gradle and settings.gradle files, do the following:

A. paste these 2 lines into the "plugins" area in build.gradle:
```kotlin
id 'org.jetbrains.kotlin.jvm' version '1.7.20-Beta'
id 'io.gitlab.arturbosch.detekt' version '1.21.0'
```

B. paste this below the includeDesktopSupport line in buld.gradle:
```kotlin
kotlin {
    compileKotlin {
        kotlinOptions {
            freeCompilerArgs += "-Xcontext-receivers"
            freeCompilerArgs += "-Xallow-kotlin-package"
            freeCompilerArgs += "-Xskip-prerelease-check"
        }
    }
}

repositories {
    mavenLocal()
    mavenCentral()
}
```

C. Paste this inside the dependencies bracket in build.gradle, below ``` implementation wpi.java.vendor.java()"```:
```implementation "com.batterystaple:ChargerLib:1.0.0-SNAPSHOT"```

D. At the end of the build.gradle file, add this:
```kotlin
detekt {
    toolVersion = "1.21.0"
    config = files("config/detekt/detekt.yml")
    buildUponDefaultConfig = true
}
```

E. In the settings.gradle file, add the following right below the mavenLocal():
mavenCentral()

### 2. Adding Detekt:
A. Add a folder called "config" in your FRC project, then in that folder add a folder named "detekt"
B. Inside of the "detekt" folder, add a file called "detekt.yml". Paste in this code into it:
```kotlin
complexity:
  LongParameterList:
    functionThreshold: 8
style:
  NewLineAtEndOfFile:
    active: false
build:
  maxIssues: -1
```


# How to deploy chargerlib

A. Download and unzip(if nessecary) chargerlib from the github page.

B. In the terminal, type .\gradlew.bat publishToMavenLocal 

C. You're done! Build and deploy your robot code, then run it.



# The Basics


### Sensor wrappers: 
The Limelight, DriverCamManager(manages driver cams plugged into roborio), navX now have been wrapped into objects for easier use.

Defined with: ```NavX()```, ```DriverCamManager(totalCameras = 3)```, ```limelight()```

NavX features: split into gyroscope, accelerometer and compass. Provides heading.

### Motor Wrappers
Wraps motors such as the neo, the falcon, etc.
Defined by using: ```neoSparkMax(canBusId = 6)``` , ```falcon(canID = 9)```


### Built-in drivetrain
Adds encoderdifferentialdrivetrain and basicdfferentialdrivetrain subsystems, which allow you to easily define drivetrains in the code.
Defined like this: 

```kotlin
private val drivetrain: EncoderDifferentialDrivetrain = sparkMaxDrivetrain(
        leftMotors = EncoderMotorControllerGroup(
            neoSparkMax(canBusId = 6),
            neoSparkMax(canBusId = 9),
        ),
        rightMotors = EncoderMotorControllerGroup(
            neoSparkMax(canBusId = 4),
            neoSparkMax(canBusId = 2),
        ),
        invertMotors = false,
        gearRatio = 1.0/10.71,
        wheelDiameter = 6.inches,
        width = 27.inches
    ) {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
```

### New interfaces:
Adds new frc-specific interfaces that help standardize types and make motors, etc. easier to use.

For example:
the HeadingProvider interface includes the drivetrain and the NavX, and allows you to consistently get the heading from these sources.
the EncoderMotorController interface includes the falcon and the neosparkmax, and allows subsystems to accept any motor with an encoder.
the encoder interface allows you to consistently get position and velocity.

### ChargerController:
Adds useful features related to the xbox controllers.
Defined by using ```ChargerController(port = 1, etc. etc.)```

