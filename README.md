# Reefscape 2025 - FRC 167 Robot Code with MapleSim integration

This repository contains the complete Java codebase for FRC Team 167, Children of the Corn, for the 2025 Reefscape competition.

This project features the physics simulation, **[MapleSim](https://shenzhen-robotics-alliance.github.io/maple-sim/)**, which we used to provide a realistic environment for driver practice.
As this was an offseason project, MapleSim was not used to develop auto routines.

## Documentation

* **[Setup Guide](MAPLESIM_SETUP.md):** Step-by-step instructions for installing and running the simulation.
* **[MapleSim Integration Details](MAPLESIM_INTEGRATION.md):** A technical breakdown of how the MapleSim physics engine is integrated into the codebase.

---

## Key Features

This codebase is built around modern FRC best practices. Here are some of the key features and where to find them:

* **MapleSim Integration:** The drivetrain and mechanisms are simulated with the MapleSim physics engine.
    * *Location:* The core logic is in `src/main/java/frc/cotc/drive/SwerveIOPhoenix.java` within the `OdometryThread` inner class.

* **IO Abstraction Layer:** The robot's control logic is cleanly separated from its hardware implementation. This allows the same logic to run on both the real robot and in simulation without any changes.
    * *Example:* The elevator's logic is in `Elevator.java`, its "contract" is in `ElevatorIO.java`, and its hardware-specific code is in `ElevatorIOPhoenix.java`.

* **Advanced Drivetrain Control:** The swerve drive uses a custom `SwerveSetpointGenerator` to ensure smooth, kinematically-valid movement, preventing infeasible commanded accelerations and minimizing wheel slip/wear. Originally by 254, modified by mjansen, further modified by 167.
    * *Location:* `src/main/java/frc/cotc/drive/SwerveSetpointGenerator.java`

* **Potential Field Pathing:** A potential field pathing algorithm is implemented in `RepulsorFieldPlanner.java`, allowing the robot to dynamically avoid obstacles when pathing to a point. In an electron potential field analogy, the target pose is an attracting force, and obstacles are repelling forces, with the sum of forces being the direction the robot should move towards.
    * *Location:* `src/main/java/frc/cotc/drive/RepulsorFieldPlanner.java`

* **Vision-Based Pose Estimation:** The robot uses AprilTag fiducial markers on the field to accurately determine its position at all times.
    * *Location:* `src/main/java/frc/cotc/vision/FiducialPoseEstimator.java`.

---

## Project File Structure

Here is a complete breakdown of the project's file structure.

<details>
<summary>Click to expand the full file tree</summary>

```
📁 2025_Reefscape-MapleSim/
│
├── 📁 ascope_assets/
│   └── 📁 Robot_KernelOverflow/
│       └── 📄 config.json        # Configuration for a custom 3D robot model used in AdvantageScope, defining its appearance and camera positions.
│
├── 📁 gradle/
│   └── 📁 wrapper/
│       ├── 📄 gradle-wrapper.jar  # The actual Gradle Wrapper executable. This allows everyone to use the same version of Gradle to build the code without installing it manually.
│       └── 📄 gradle-wrapper.properties # Configuration for the Gradle Wrapper, specifying which version of Gradle to download and use.
│
├── 📁 src/main/
│   ├── 📁 deploy/
│   │   ├── 📄 2025-reefscape-welded-reefonly.json # A map of all the AprilTag locations on the 2025 FRC field, used by the vision system to determine the robot's position.
│   │   └── 📄 example.txt         # A placeholder file demonstrating that any files in this directory will be copied to the RoboRIO during deployment.
│   │
│   └── 📁 java/frc/cotc/
│       ├── 📄 Main.java           # The main entry point for the Java program. Its sole responsibility is to start the `Robot` class.
│       ├── 📄 Robot.java          # The central hub of the entire robot program. It initializes all subsystems, sets up controller bindings, and manages the overall robot state (teleop, auto, etc.).
│       ├── 📄 Constants.java      # A collection of important, robot-wide numerical constants, such as the robot's physical dimensions and field measurements.
│       ├── 📄 Autos.java          # This class defines all the autonomous routines. It uses a chooser to select which auto to run based on Driver Station input.
│       │
│       ├── 📁 drive/             # Contains all code related to the swerve drivetrain.
│       │   ├── 📄 Swerve.java             # High-level control logic for the swerve drive. It translates driver inputs or auto paths into chassis speeds and manages pose estimation, but does not directly interface with motors.
│       │   ├── 📄 SwerveIO.java           # The "contract" or interface for the swerve drive. It defines the required methods and data structures that any specific hardware implementation must provide.
│       │   ├── 📄 SwerveIOPhoenix.java    # The hardware-specific implementation of `SwerveIO` using CTRE Phoenix 6 libraries to control the Kraken motors and Pigeon 2 gyro.
│       │   ├── 📄 SwervePoseEstimator.java # A custom version of WPILib's pose estimator, which fuses sensor data from the gyro and wheel encoders to track the robot's position on the field.
│       │   ├── 📄 SwerveSetpointGenerator.java # A sophisticated class that ensures smooth and kinematically possible transitions between swerve drive states, preventing wheel slip and instability.
│       │   └── 📄 RepulsorFieldPlanner.java # Implements a pathfinding algorithm that uses "repulsor fields" to navigate around obstacles on the field, like the Reef.
│       │
│       ├── 📁 superstructure/     # Contains code for all robot mechanisms other than the drivetrain.
│       │   ├── 📄 Superstructure.java     # Acts as a coordinator for all the other subsystems in this package, creating commands that involve multiple mechanisms working together.
│       │   ├── 📄 AlgaeClaw.java          # Logic for the Algae Claw, combining the pivot and rollers into a single functional unit.
│       │   ├── 📄 AlgaePivot.java         # Logic for the pivoting motion of the Algae Claw.
│       │   ├── 📄 AlgaePivotIO.java       # Interface for the Algae Pivot.
│       │   ├── 📄 AlgaePivotIOPhoenix.java # Phoenix implementation for the Algae Pivot.
│       │   ├── 📄 AlgaePivotIOSim.java    # Simulation implementation for the Algae Pivot.
│       │   ├── 📄 AlgaeRollers.java       # Logic for the rollers that intake and eject Algae.
│       │   ├── 📄 AlgaeRollersIO.java     # Interface for the Algae Rollers.
│       │   ├── 📄 AlgaeRollersIOPhoenix.java # Phoenix implementation for the Algae Rollers.
│       │   ├── 📄 AlgaeRollersIOSim.java  # Simulation implementation for the Algae Rollers.
│       │   ├── 📄 Climber.java            # Logic for the climbing mechanism.
│       │   ├── 📄 ClimberIO.java          # Interface for the Climber.
│       │   ├── 📄 ClimberIOPhoenix.java   # Phoenix implementation for the Climber.
│       │   ├── 📄 CoralOuttake.java       # Logic for the Coral scoring mechanism.
│       │   ├── 📄 CoralOuttakeIO.java     # Interface for the Coral Outtake.
│       │   ├── 📄 CoralOuttakeIOPhoenix.java # Phoenix implementation for the Coral Outtake.
│       │   ├── 📄 CoralOuttakeIOSim.java  # Simulation implementation for the Coral Outtake.
│       │   ├── 📄 Elevator.java           # Logic for the elevator mechanism.
│       │   ├── 📄 ElevatorIO.java         # Interface for the Elevator.
│       │   ├── 📄 ElevatorIOPhoenix.java  # Phoenix implementation for the Elevator.
│       │   ├── 📄 Ramp.java               # Logic for the deployable ramp.
│       │   ├── 📄 RampIO.java             # Interface for the Ramp.
│       │   └── 📄 RampIOPhoenix.java      # Phoenix implementation for the Ramp.
│       │
│       ├── 📁 util/               # A collection of helper classes and utilities.
│       │   ├── 📄 CommandXboxControllerWithRumble.java # An extension of the standard Xbox controller class that adds a convenient command for rumbling the controller.
│       │   ├── 📄 ContinuousElevatorSim.java # A simulation class specifically for a continuous-style elevator with multiple stages.
│       │   ├── 📄 FOCMotorSim.java         # A physics simulation class for FOC (Field-Oriented Control) motors, which models current instead of voltage.
│       │   ├── 📄 GainsCalculator.java     # A utility to calculate optimal P and D gains for a PID controller based on motor characteristics.
│       │   ├── 📄 Mechanism.java         # A base class that allows multiple subsystems to be grouped and treated as a single unit, simplifying command requirements.
│       │   ├── 📄 MotorCurrentDraws.java  # A simple data structure for holding the stator and supply current of a motor.
│       │   ├── 📄 PhoenixBatchRefresher.java # An optimization class that batches calls to Phoenix devices to improve performance on the RoboRIO by reducing overhead.
│       │   └── 📄 ReefLocations.java     # A utility class that defines the precise 2D coordinates of all scoring locations and other key points on the Reef structure.
│       │
│       └── 📁 vision/             # Code related to the robot's vision system.
│           ├── 📄 FiducialPoseEstimator.java      # The main logic for estimating the robot's pose using AprilTag fiducials, combining data from multiple cameras.
│           ├── 📄 FiducialPoseEstimatorIO.java    # The "contract" or interface for a fiducial-based pose estimation system, defining what data it must provide.
│           └── 📄 FiducialPoseEstimatorIOPhoton.java # An implementation of the vision IO interface using the PhotonVision library to get data from the cameras.
│
├── 📁 vendordeps/             # Vendor-supplied libraries.
│   ├── 📄 AdvantageKit.json     # Configuration for AdvantageKit, a logging and data visualization framework.
│   ├── 📄 maple-sim.json        # Configuration for MapleSim, a physics simulation engine.
│   ├── 📄 Phoenix6-25.3.1.json  # Configuration for the CTRE Phoenix 6 library, which is used to control modern CTRE hardware like Kraken motors.
│   ├── 📄 photonlib-v2025.3.1-rc1.json # Configuration for PhotonVision, the library used for AprilTag detection.
│   └── 📄 WPILibNewCommands.json # Configuration for the WPILib command-based framework.
│
├── 📄 .gitattributes          # A Git configuration file that ensures consistent line endings across different operating systems.
├── 📄 .gitignore              # A list of files and folders that Git should ignore and not track (e.g., build artifacts, user settings).
├── 📄 build.gradle            # The master script for Gradle, the build system. It defines dependencies, plugins, and tasks for building, testing, and deploying the code.
├── 📄 gradlew                 # A shell script for executing Gradle tasks on Linux and macOS.
├── 📄 gradlew.bat             # A batch script for executing Gradle tasks on Windows.
├── 📄 LICENSE                 # The MIT License file, which specifies the permissions and limitations for using this software.
├── 📄 README.md               # The main documentation for this project, explaining how to set up and run the simulation.
├── 📄 settings.gradle         # Configuration settings for the Gradle build, such as defining where to find plugins.
├── 📄 simgui-ds.json          # Stores the configuration for the simulation GUI's virtual joysticks and driver station layout.
├── 📄 simgui-ds.json.example  # An example configuration file for the simulation GUI.
└── 📄 WPILib-License.md       # The license specific to the WPILib libraries used in the project.
```

</details>