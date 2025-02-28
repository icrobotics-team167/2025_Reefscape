// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttake extends SubsystemBase {

  private final CoralOuttakeIO io;
  private final CoralOuttakeIO.CoralOuttakeIOInputs inputs =
      new CoralOuttakeIO.CoralOuttakeIOInputs();

  public CoralOuttake(CoralOuttakeIO io) {
    this.io = io;
  }

  public Command in() {
    return runOnce(() -> io.runVoltage(5));
  }

  public Command out() {
    return runOnce(() -> io.runVoltage(8));
  }

  public Command stop() {
    return runOnce(() -> io.runVoltage(0));
  }
}
