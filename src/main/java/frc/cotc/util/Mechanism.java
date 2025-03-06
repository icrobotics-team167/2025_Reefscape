// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A way to nest {@link edu.wpi.first.wpilibj2.command.Subsystem}s such that from the outside, it
 * only looks like one Subsystem, while still maintaining default command and command requirement
 * behaviors for the nested subsystems.
 *
 * <p><b>All PUBLICLY AVAILABLE COMMANDS MUST BE RUN THROUGH {@link Mechanism#expose(Command)} OR
 * ELSE IT'S ENTIRELY UNDEFINED BEHAVIOR.
 */
public class Mechanism extends SubsystemBase {
  /**
   * Wraps a command such that from the outside, it looks like only the parent subsystem is
   * requiring the command.
   *
   * <p><b>All PUBLICLY AVAILABLE COMMANDS MUST BE RUN THROUGH THIS METHOD OR * ELSE IT'S ENTIRELY
   * UNDEFINED BEHAVIOR.
   *
   * @param internal The command to wrap
   * @return The wrapped command.
   */
  protected Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }
}
