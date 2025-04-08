// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIO io;
  private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();

  AlgaePivot(AlgaePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Algae/Pivot", inputs);
  }

  Command intake() {
    return goToAngle(Units.degreesToRadians(-35)).withName("Intake");
  }

  Command stow() {
    return goToAngle(Units.degreesToRadians(-75)).withName("Stow");
  }

  Command hold() {
    return goToAngle(Units.degreesToRadians(110)).withName("Hold");
  }

  @AutoLogOutput(key = "Superstructure/Algae/Pivot/Target Angle")
  private double targetAngle = Units.degreesToRadians(-75);

  private Command goToAngle(double angleRad) {
    return runOnce(() -> targetAngle = angleRad).andThen(run(() -> io.goToAngle(angleRad)));
  }

  boolean atTargetAngle() {
    return MathUtil.isNear(targetAngle, inputs.posRad, Units.degreesToRadians(5))
        && Math.abs(inputs.velRadPerSec) < Units.degreesToRadians(10);
  }
}
