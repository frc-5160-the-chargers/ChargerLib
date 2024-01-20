# ChargerLib

Notice: ChargerLib itself has officially been converted to a "lib" folder within yearly repos; see FRC2024 for the latest version.
In the future, the command DSL and advantagekit integration within chargerlib will be moved to maven central repositories.



Some features include:
- Classes for some common subsystems (e.g. drivetrains)
- Support for units in all values provided through [KMeasure](https://github.com/battery-staple/KMeasure)
- These units also take advantage of kotlin inline classes, which means that during runtime, they are mostly represented as double's: this reduces runtime overhead.
- Units-based wrappers for most WPILib classes, including Pose2d, Pose3d, Trapezoid Profiling, and more.

- Wrapping of components from different manufacturers to allow better subsystem compatibility and hardware abstraction (for example, switching a drivetrain from NEOs to Falcons becomes trivial)
- Idiomatic Kotlin Commands DSL for creating commands and autonomous routines
- And a custom AdvantageKit wrapper that takes advantage of kotlin property delegates to create auto-logged inputs!

In order to see installation, documentation, and more, visit the wiki and the [Github pages site](https://frc-5160-the-chargers.github.io/ChargerLib/)
