// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.algae;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  class AlgaeIntakeIOInputs {
    boolean hasAlgae;

    MotorCurrentDraws currentDraws = new MotorCurrentDraws();
  }

  default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  default void intake() {}

  default void hold() {}

  default void outtake() {}

  default void brake() {}
}
