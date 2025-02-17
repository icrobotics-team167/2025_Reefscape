// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralOuttake extends SubsystemBase {
  private final CoralOuttakeIO io;
  private final CoralOuttakeIO.CoralOuttakeIOInputs inputs =
      new CoralOuttakeIO.CoralOuttakeIOInputs();

  CoralOuttake(CoralOuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/CoralOuttake", inputs);
  }

  Command intake() {
    return run(io::intake).until(() -> inputs.hasCoral).finallyDo(io::brake);
  }

  Command score() {
    return run(io::outtake).onlyWhile(() -> inputs.hasCoral).withTimeout(1).finallyDo(io::brake);
  }
}
