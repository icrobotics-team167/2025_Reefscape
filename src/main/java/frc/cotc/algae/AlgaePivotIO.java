// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.algae;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
  @AutoLog
  class AlgaeClawIOInputs {
    double posRad;
    double velRadPerSec;

    MotorCurrentDraws motorCurrentDraws = new MotorCurrentDraws();
  }

  default void updateInputs(AlgaeClawIOInputs inputs) {}

  default void setTargetPos(double angleRad) {}

  default void manualOverride(double voltage) {}

  default void resetAlgae() {}
}
