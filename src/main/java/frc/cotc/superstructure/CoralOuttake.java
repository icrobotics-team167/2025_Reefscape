// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

class CoralOuttake extends SubsystemBase {
  private final CoralOuttakeIO io;
  private final CoralOuttakeIOInputsAutoLogged inputs = new CoralOuttakeIOInputsAutoLogged();

  CoralOuttake(CoralOuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/CoralOuttake", inputs);
  }

  Command intake() {
    return run(io::intake).until(() -> inputs.hasCoral).withName("Intake");
  }

  Command scoreFast() {
    return run(io::outtakeFast)
        .withDeadline(waitUntil(() -> !inputs.hasCoral).andThen(waitSeconds(.15)))
        .withName("Outtake Fast");
  }

  Command scoreSlow() {
    return run(io::outtakeSlow)
        .withDeadline(waitUntil(() -> !inputs.hasCoral).andThen(waitSeconds(.15)))
        .withName("Outtake Slow");
  }

  Command scoreSuperSlow() {
    return run(io::outtakeSuperSlow)
        .withDeadline(waitUntil(() -> !inputs.hasCoral).andThen(waitSeconds(.15)))
        .withName("Outtake Super Slow");
  }

  Command agitate() {
    return run(io::agitate).withName("Agitate");
  }

  boolean hasCoral() {
    return inputs.hasCoral;
  }

  boolean coralStuck() {
    return inputs.velocityPercent < .1 && inputs.currentDraws.statorCurrent > 15;
  }
}
