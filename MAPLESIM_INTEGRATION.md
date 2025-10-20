# MapleSim Technical Integration

This document provides a technical overview of how the MapleSim physics engine is integrated into the FRC 167 robot codebase. As well as providing some instruction 

---

## Overview

The core principle of this simulation is the **IO Abstraction Layer**. The main robot logic (`Swerve.java`, `Elevator.java`, etc.) is written to an interface (`SwerveIO.java`, `ElevatorIO.java`). For simulation, we provide a special implementation of that interface (`*Sim.java`) that communicates with the physics engine instead of real hardware. The `Robot.java` class chooses which implementation to use based on whether the code is running in a simulation environment.

---

