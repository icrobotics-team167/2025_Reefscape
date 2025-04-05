// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
  @AutoLog
  class AlgaePivotIOInputs {
    double posRad;
    double velRadPerSec;
    MotorCurrentDraws currentDraws = new MotorCurrentDraws();
  }

  default void updateInputs(AlgaePivotIOInputs inputs) {}

  default void intake() {}

  default void stow() {}

  default void hold() {}
}
