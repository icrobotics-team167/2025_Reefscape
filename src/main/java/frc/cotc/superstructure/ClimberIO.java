// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    double posRad;

    MotorCurrentDraws currentDraws = new MotorCurrentDraws();
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void stop() {}

  default void deploy() {}

  default void climb() {}
}
