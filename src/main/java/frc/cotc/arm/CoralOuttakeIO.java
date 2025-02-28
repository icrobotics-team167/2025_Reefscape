// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface CoralOuttakeIO {
  class CoralOuttakeIOInputs implements LoggableInputs {

    public boolean coralIn = false;
    public MotorCurrentDraws motorCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable logTable) {
      logTable.put("coralIn", coralIn);
      logTable.put("motorCurrentDraws", MotorCurrentDraws.struct, motorCurrentDraws);
    }

    @Override
    public void fromLog(LogTable logTable) {}
  }

  void updateInputs(CoralOuttakeIOInputs inputs);

  void runVoltage(double volts);

  void brake();
}
