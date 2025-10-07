# Reefscape 2025 Robot Code with MapleSim Physics Simulation

## 1. Intro

This document provides instructions for the `maplesim` branch of the 2025 Reefscape robot code for FRC 167 Children of the Corn. This version integrates the MapleSim physics engine to make the simulations more realistic to real world robot behavior.

The primary purpose of this simulation setup is to provide a realistic practice environment for the drive team, by using physics-based simulations that arent in the standard simulation available on the `main` branch.

---

## 2. Prerequisites

The following software must be installed on your system before proceeding:

* **GitHub Desktop**: For cloning and managing the source code repository.
* **WPILib 2025 for VSCode**: The FRC-specific integrated development environment.

---

## 3. Installation Instructions

### Clone the Repository

1.  Navigate to the team's 2025 Reefscape GitHub repository.
2.  Ensure the `maplesim` branch is selected from the branch dropdown menu.
3.  Click the **`< > Code`** button and copy the HTTPS URL from this popup.
4.  Open GitHub Desktop.
5.  From the menu bar, select **File** > **Clone Repository...**.
6.  In the clone dialog, select the **URL** tab, paste the repository link, choose a local directory for the project, and click **Clone**.

The project source code will be downloaded to the specified local path.

---

## 4. Running the Simulation

After cloning the repository, open the project folder in 2025 WPILib VS Code to begin.

### Launch the Simulator

1.  In VSCode, open the command palette using the shortcut `Ctrl+Shift+P`.
2.  Type `>Simulate Robot Code` into the command palette and press **Enter**.
3.  In the confirmation pop-up, check the box labeled **Sim GUI**, then click **OK**. 
    - Note, it may take a bit for this popup to appear. It will appear at the top of the window.
4.  The simulation GUI will launch, providing an interface to control the robot's state and monitor data.

### Configure Controller

1.  Connect an Xbox One controller to your computer.
2.  In the Sim GUI window, locate the **System Joysticks** list.
3.  Drag your controller from the **System Joysticks** list and drop it onto the **`Joystick[0]`** entry in the **Joystick** panel.
Below is a description of the different joystick options available.
   * `Joystick[0]` controls the drivetrain.
   * `Joystick[1]` controls the elevator and game piece mechanisms.
To fully operate the robot, you will need an additional controller for `Joystick[1]`.

---

## 5. AdvantageScope Configuration

AdvantageScope is used for 3D visualization of data. Here we will use ot to visalize the robot and the field.

### Connect to the Simulator

1.  Launch AdvantageScope.
2.  From the menu bar, select **File** -> **Connect to Simulator**.
3.  In the bottom-right corner of the window, use the dropdown menu to select the **2025 Reefscape Field**.
4.  To adjust the camera perspective, right-click within the 3D field view. Available options include:
    * **Orbit Field**: Provides a comprehensive view of the entire field.
    * **Orbit Robot**: The camera follows the robot's position.
    * **Driver Station**: Offers views from specific driver station locations (e.g., `Red1`).
    * **Set FOV**: I like to set this to 90 or 100. I think 90deg is the closest to realistic.

> **Note**: The robot's starting location is always on the blue side.
> The red/blue relative controls are configured within the Sim GUI, not through AdvantageScope.
    - The Sim GUI defaults to Red1, so you can set your driver station to here if you like.
    - If you want to change this, you can do so from the FMS tab in the Sim GUI.
    - Otherwise, orient the camera to your liking via the Orbit Field tool.

### Add Simulation Objects to the 3D View

To render the robot and game pieces, their poses must be added to the 3D view.

1.  In the left-hand sidebar, expand the tree: `AdvantageScope` -> `RealOutputs` -> `Sim`.
2.  Drag each object from the `Sim` sub-tab and drop it into the poses area of the main 3D view.. This will populate the field with our gamepieces and robot. The required objects are:
    * `Barge Algae`
    * `Coral`
    * `Ground Algae`
    * `Ground Truth Pose` (This represents the robot's chassis)
    * `Reef Algae`
    * `Reef Coral`
    * `Robot Algae`
    * `Robot Coral`

### Add the Elevator Visualization

1.  In the left-hand sidebar, navigate to `RealOutputs` -> `SuperStructure` -> `Elevator`.
2.  Drag the `Visualization` object from this location and drop it directly onto the `Ground Truth Pose` entry in the **Poses** list. This attaches the elevator mechanism to the robot's chassis in the 3D view.

### Define Game Piece Models

Assign the correct 3D models to the objects added in the previous step.

1.  For each of the following poses, click the green cube icon next to its name in the **Poses** list, then select **Game Piece** -> **Algae**:
    * `RealOutputs/Sim/Ground Algae`
    * `RealOutputs/Sim/Barge Algae`
    * `RealOutputs/Sim/Reef Algae`
    * `RealOutputs/Sim/Robot Algae`
2.  For each of the following poses, use the same method to select **Game Piece** -> **Coral**:
    * `RealOutputs/Sim/Coral`
    * `RealOutputs/Sim/Robot Coral`
    * `RealOutputs/Sim/Reef Coral`

---

## 6. Importing the Custom Robot Model

Follow these steps to replace the default robot model with the team's custom asset.

1.  In AdvantageScope, open the application's assets folder by navigating to **Help** -> **Show Assets Folder**.
2.  From your local repository folder, locate the `ascope_assets` directory.
3.  Copy the `Robot_KernelOverflow` folder from `ascope_assets` and paste it into the `userAssets` folder you opened in the previous step.
4.  Return to AdvantageScope. In the **Poses** list, find the `Ground Truth Pose` entry.
5.  Click the robot icon next to its name and select `Kernel Overflow` from the dropdown list of models.

The simulation setup is now complete.
To drive the robot, enable it in the Sim GUI by selecting `Teleop` from the box at the top right. Make sure the Sim GUI is the active window as well.
