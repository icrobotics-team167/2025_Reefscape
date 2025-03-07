// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIO io;
  private final AlgaePivotIO.AlgaeClawIOInputs inputs = new AlgaePivotIO.AlgaeClawIOInputs();

  public AlgaePivot(AlgaePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/AlgaeClaw", inputs);
  }

  public Command manualControl(DoubleSupplier overrideControl) {
    return run(() -> io.manualOverride(overrideControl.getAsDouble() * 4));
  }
}
