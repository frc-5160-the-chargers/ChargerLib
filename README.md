# ChargerLib

ChargerLib is the backbone of all FRC code for the FRC 5160 team(at least so far). It adds a lot of useful features that make robot coding easier, simpler and better overall.
Unfortunately, there's no docs for chargerlib so far; however, this readme will give you the basics about it.


# Gradle setup in your FRC projects

In order to use ChargerLib, you're going to have to add lines of code into the build.gradle and settings.gradle parts of your FRC ROBOT code. 
You do not have to edit chargerlib at all; it should always be set up to work.

### 1. In your build.gradle and settings.gradle files, do the following:

A. paste these 2 lines into the "plugins" area in build.gradle:
```kotlin
id 'org.jetbrains.kotlin.jvm' version '1.9.20-Beta'
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

In order to use the library, check the Chargerlib Wiki.

