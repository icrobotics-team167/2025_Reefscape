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

public interface AlgaePivotIO {
  class AlgaeClawIOInputs implements LoggableInputs {
    double posRad;
    double velRadPerSec;

    MotorCurrentDraws motorCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("posRad", posRad);
      table.put("velRadPerSec", velRadPerSec);
      table.put("motorCurrentDraws", MotorCurrentDraws.struct, motorCurrentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      posRad = table.get("posRad", 0.0);
      velRadPerSec = table.get("velRadPerSec", 0.0);
      motorCurrentDraws =
          table.get("motorCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  default void updateInputs(AlgaeClawIOInputs inputs) {}

  default void setTargetPos(double angleRad) {}

  default void manualOverride(double volts) {}
}
