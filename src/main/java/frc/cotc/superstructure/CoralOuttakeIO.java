// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface CoralOuttakeIO {
  class CoralOuttakeIOInputs implements LoggableInputs {
    boolean hasCoral;

    double velocityPercent;

    MotorCurrentDraws currentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("hasCoral", hasCoral);
      table.put("velocityPercent", velocityPercent);
      table.put("currentDraws", MotorCurrentDraws.struct, currentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      hasCoral = table.get("hasCoral", false);
      velocityPercent = table.get("velocityPercent", 0.0);
      currentDraws = table.get("currentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  default void updateInputs(CoralOuttakeIOInputs inputs) {}

  default void intake() {}

  default void outtake() {}

  default void agitate() {}

  default void brake() {}
}
