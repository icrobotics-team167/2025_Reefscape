// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandXboxControllerWithRumble extends CommandXboxController {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandXboxControllerWithRumble(int port) {
    super(port);
  }

  public Command rumble(double seconds) {
    return sequence(
        runOnce(() -> setRumble(GenericHID.RumbleType.kBothRumble, 1)),
        waitSeconds(seconds),
        runOnce(() -> setRumble(GenericHID.RumbleType.kBothRumble, 0)));
  }
}
