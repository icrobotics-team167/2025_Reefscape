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

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

  Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Climber", inputs);
  }

  Command deployStart() {
    return run(io::deploy)
        .until(() -> inputs.posRad < Units.degreesToRadians(45))
        .finallyDo(io::stop);
  }

  Command deploy() {
    return run(io::deploy).finallyDo(io::stop);
  }

  Command climb() {
    return run(io::climb).finallyDo(io::stop);
  }
}
