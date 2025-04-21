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

class AlgaeRollers extends SubsystemBase {
  private final AlgaeRollersIO io;
  private final AlgaeRollersIOInputsAutoLogged inputs = new AlgaeRollersIOInputsAutoLogged();

  AlgaeRollers(AlgaeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Algae/Rollers", inputs);
  }

  Command intake() {
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

  Command eject() {
    return run(io::eject).withTimeout(.2).finallyDo(io::brake).withName("Eject");
  }

  boolean hasAlgae() {
    return inputs.hasAlgae;
  }
}
