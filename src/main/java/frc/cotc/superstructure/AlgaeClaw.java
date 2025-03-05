// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

class AlgaeClaw extends SubsystemBase {
  private final AlgaeClawIO io;
  private final AlgaeClawIO.AlgaeClawIOInputs inputs = new AlgaeClawIO.AlgaeClawIOInputs();

  AlgaeClaw(AlgaeClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/AlgaeClaw", inputs);
  }

  Command intake() {
    return run(() -> io.setTargetPos(Units.degreesToRadians(-40))).withName("Intake");
  }

  Command processor() {
    return run(() -> io.setTargetPos(Units.degreesToRadians(-50))).withName("Processor");
  }

  Command barge() {
    return run(() -> io.setTargetPos(Units.degreesToRadians(45))).withName("Barge");
  }

  Command stow() {
    return run(() -> io.setTargetPos(Units.degreesToRadians(-75))).withName("Stow");
  }
}
