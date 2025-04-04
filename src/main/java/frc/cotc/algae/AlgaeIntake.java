// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/AlgaeIntake", inputs);
  }

  public Command intake() {
    return run(() -> {
          if (inputs.hasAlgae) {
            io.hold();
          } else {
            io.intake();
          }
        })
        .finallyDo(io::brake)
        .withName("Intake");
  }

  public Command outtake() {
    return run(io::outtake).finallyDo(io::brake).withName("Outtake");
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }
}
